#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

namespace arm_ik_kdl
{
using Pose = geometry_msgs::msg::Pose;

KDL::Frame toKdl(const Pose & p)
{
  return KDL::Frame(
    KDL::Rotation::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
    KDL::Vector(p.position.x, p.position.y, p.position.z));
}

Pose fromKdl(const KDL::Frame & f)
{
  Pose p;
  p.position.x = f.p.x(); p.position.y = f.p.y(); p.position.z = f.p.z();
  f.M.GetQuaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
  return p;
}

struct Arm
{
  std::string side;
  std::string base;
  std::string tip;
  KDL::Chain chain;
  KDL::JntArray q;
  KDL::JntArray q_min;
  KDL::JntArray q_max;
  std::vector<std::string> joint_names;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos;
  KDL::Frame target;
  bool have_target{false};
  bool have_state{false};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr delta_sub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr solution_pub;
};

class HumanoidIkNode final : public rclcpp::Node
{
public:
  HumanoidIkNode() : Node("arm_ik_kdl")
  {
    const auto description = declare_parameter<std::string>("robot_description", "");
    if (description.empty()) {
      throw std::runtime_error(
        "Parameter 'robot_description' is empty. Pass the URDF XML or remap/launch it from robot_state_publisher.");
    }
    urdf::Model model;
    if (!model.initString(description)) {
      throw std::runtime_error("Failed to parse robot_description as URDF.");
    }
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(description, tree)) {
      throw std::runtime_error("kdl_parser could not construct a KDL tree from robot_description.");
    }

    const auto base = declare_parameter<std::string>("base_link", "waist_yaw_link");
    createArm(tree, model, "left", base, "left_tcp_link");
    createArm(tree, model, "right", base, "right_tcp_link");

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>("joint_states_topic", "/joint_states"),
      rclcpp::SensorDataQoS(),
      std::bind(&HumanoidIkNode::onJointState, this, std::placeholders::_1));

    const auto hz = declare_parameter<double>("update_rate", 100.0);
    if (hz <= 0.0) {throw std::runtime_error("update_rate must be > 0");}
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / hz),
      std::bind(&HumanoidIkNode::update, this));
    RCLCPP_INFO(get_logger(), "KDL IK ready: base=%s, 7-DOF left/right chains", base.c_str());
  }

private:
  void createArm(
    const KDL::Tree & tree, const urdf::Model & model,
    const std::string & side, const std::string & base, const std::string & tip)
  {
    auto arm = std::make_unique<Arm>();
    arm->side = side; arm->base = base; arm->tip = tip;
    if (!tree.getChain(base, tip, arm->chain)) {
      throw std::runtime_error("No KDL chain from " + base + " to " + tip);
    }
    const unsigned int n = arm->chain.getNrOfJoints();
    if (n != 7) {
      throw std::runtime_error(side + " arm expected 7 movable joints, found " + std::to_string(n));
    }
    arm->q = KDL::JntArray(n);
    arm->q_min = KDL::JntArray(n);
    arm->q_max = KDL::JntArray(n);
    for (const auto & segment : arm->chain.segments) {
      const auto & joint = segment.getJoint();
      if (joint.getType() == KDL::Joint::None) {continue;}
      const auto urdf_joint = model.getJoint(joint.getName());
      if (!urdf_joint || !urdf_joint->limits) {
        throw std::runtime_error("Missing URDF limits for joint " + joint.getName());
      }
      const auto i = arm->joint_names.size();
      arm->joint_names.push_back(joint.getName());
      arm->q_min(i) = urdf_joint->limits->lower;
      arm->q_max(i) = urdf_joint->limits->upper;
    }
    arm->fk = std::make_unique<KDL::ChainFkSolverPos_recursive>(arm->chain);
    arm->ik_vel = std::make_unique<KDL::ChainIkSolverVel_pinv>(arm->chain, 1e-5, 150);
    arm->ik_pos = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
      arm->chain, arm->q_min, arm->q_max, *arm->fk, *arm->ik_vel, 150, 1e-5);

    const auto prefix = "/" + side + "_arm";
    arm->pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
      prefix + "/target_pose", 10,
      [this, side](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {onPose(side, *msg);});
    arm->delta_sub = create_subscription<geometry_msgs::msg::TransformStamped>(
      prefix + "/target_delta", 10,
      [this, side](geometry_msgs::msg::TransformStamped::ConstSharedPtr msg) {onDelta(side, *msg);});
    arm->solution_pub = create_publisher<sensor_msgs::msg::JointState>(prefix + "/ik_solution", 10);
    arms_.emplace(side, std::move(arm));
  }

  Arm & getArm(const std::string & side) {return *arms_.at(side);}

  void onJointState(const sensor_msgs::msg::JointState & msg)
  {
    std::unordered_map<std::string, double> positions;
    for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
      positions[msg.name[i]] = msg.position[i];
    }
    for (auto & entry : arms_) {
      auto & arm = *entry.second;
      bool complete = true;
      for (size_t i = 0; i < arm.joint_names.size(); ++i) {
        const auto it = positions.find(arm.joint_names[i]);
        if (it == positions.end()) {complete = false; break;}
        arm.q(i) = std::clamp(it->second, arm.q_min(i), arm.q_max(i));
      }
      if (!complete) {continue;}
      arm.have_state = true;
      if (!arm.have_target) {
        KDL::Frame current;
        if (arm.fk->JntToCart(arm.q, current) >= 0) {
          arm.target = current;
          arm.have_target = true;
        }
      }
    }
  }

  void onPose(const std::string & side, const geometry_msgs::msg::PoseStamped & msg)
  {
    auto & arm = getArm(side);
    const auto & p = msg.pose;
    const double qn = p.orientation.x * p.orientation.x + p.orientation.y * p.orientation.y +
      p.orientation.z * p.orientation.z + p.orientation.w * p.orientation.w;
    if (!std::isfinite(qn) || qn < 1e-12 ||
      !std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !std::isfinite(p.position.z))
    {
      RCLCPP_WARN(get_logger(), "%s: rejected non-finite or invalid target pose", side.c_str());
      return;
    }
    arm.target = toKdl(p);
    arm.have_target = true;
  }

  void onDelta(const std::string & side, const geometry_msgs::msg::TransformStamped & msg)
  {
    auto & arm = getArm(side);
    if (!arm.have_target) {
      RCLCPP_WARN(get_logger(), "%s: delta ignored until current FK target is initialized", side.c_str());
      return;
    }
    const auto & t = msg.transform.translation;
    const auto & r = msg.transform.rotation;
    const double qn = r.x * r.x + r.y * r.y + r.z * r.z + r.w * r.w;
    if (!std::isfinite(qn) || qn < 1e-12 ||
      !std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z))
    {
      RCLCPP_WARN(get_logger(), "%s: rejected invalid delta", side.c_str());
      return;
    }
    const KDL::Frame delta(
      KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w),
      KDL::Vector(t.x, t.y, t.z));
    // Delta is expressed in the TCP-local frame: post-multiply the held target.
    arm.target = arm.target * delta;
  }

  void update()
  {
    for (auto & entry : arms_) {
      auto & arm = *entry.second;
      if (!arm.have_state || !arm.have_target) {continue;}
      KDL::JntArray solution(arm.chain.getNrOfJoints());
      const int rc = arm.ik_pos->CartToJnt(arm.q, arm.target, solution);
      if (rc < 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000, "%s IK failed (KDL code %d); no command published",
          arm.side.c_str(), rc);
        continue;
      }
      sensor_msgs::msg::JointState out;
      out.header.stamp = now();
      out.name = arm.joint_names;
      out.position.resize(arm.joint_names.size());
      for (size_t i = 0; i < arm.joint_names.size(); ++i) {out.position[i] = solution(i);}
      arm.solution_pub->publish(out);
    }
  }

  std::unordered_map<std::string, std::unique_ptr<Arm>> arms_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace arm_ik_kdl

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<arm_ik_kdl::HumanoidIkNode>());
  } catch (const std::exception & e) {
    std::cerr << "arm_ik_kdl fatal: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
