// hand_tracker_node.cpp
// Нода-источник цели «собственная рука робота» для системы фокусировки (nssr-sim, ROS 2 Humble).
//
// Идея: позиция кисти известна точно из кинематики — не нужно зрение. Нода через TF2
// берёт трансформ camera_head_link <- wrist_roll_{l,r}_link, публикует его как цель
// /focus/hand (PoseStamped) и дополнительно считает видимость руки в кадре (frustum test)
// и проекцию в пиксели (для отладки/оверлеев).
//
// Математика (см. docs/FOCUS_TRACKING.md, раздел «Слежение за рукой через TF»):
//   p_cam = T(camera_head_link <- wrist_link).transform(p_wrist)   // одна матрица 4x4
//   видима  <=>  Z > z_near && |X| <= Z*tan(hfov/2) && |Y| <= Z*tan(vfov/2)
//   пиксели: u = fx*X/Z + cx, v = fy*Y/Z + cy
//
// Интерфейсы:
//   ВХОД  /tf, /tf_static            tf2_msgs/TFMessage (robot_state_publisher из /joint_states)
//   ВХОД  /cam_head/color/camera_info sensor_msgs/CameraInfo
//   ВЫХОД /focus/hand                geometry_msgs/PoseStamped  frame_id=camera_head_link
//   ВЫХОД /focus/hand_visible        std_msgs/Bool              true, если кисть в поле зрения
//   СЕРВИС /focus/set_hand           std_srvs/SetBool           выбрать руку: true=правая, false=левая
//
// Запуск:
//   ros2 run focus_tracking hand_tracker_node --ros-args -p use_sim_time:=true \
//     -p hand:=right -p rate_hz:=50.0

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class HandTrackerNode : public rclcpp::Node
{
public:
    HandTrackerNode()
    : Node("hand_tracker_node")
    {
        camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_head_link");
        hand_         = declare_parameter<std::string>("hand", "right");   // left | right
        rate_hz_      = declare_parameter<double>("rate_hz", 50.0);
        z_near_       = declare_parameter<double>("z_near", 0.15);          // м, min depth камеры

        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/focus/hand", rclcpp::QoS(10));
        visible_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/focus/hand_visible", rclcpp::QoS(10));

        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/cam_head/color/camera_info", rclcpp::QoS(5),
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr m) {
                fx_ = m->k[0]; fy_ = m->k[4];
                cx_ = m->k[2]; cy_ = m->k[5];
                got_intrinsics_ = true;
            });

        set_hand_srv_ = create_service<std_srvs::srv::SetBool>(
            "/focus/set_hand",
            [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                   std_srvs::srv::SetBool::Response::SharedPtr res) {
                hand_ = req->data ? "right" : "left";
                res->success = true;
                res->message = "tracking hand: " + hand_;
                RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
            });

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / rate_hz_)),
            std::bind(&HandTrackerNode::tick, this));

        RCLCPP_INFO(get_logger(), "hand_tracker_node: следим за '%s' рукой в рамке '%s', %.0f Гц",
                    hand_.c_str(), camera_frame_.c_str(), rate_hz_);
    }

private:
    std::string wrist_frame() const
    {
        return "wrist_roll_" + hand_ + "_link";
    }

    void tick()
    {
        geometry_msgs::msg::TransformStamped ts;
        try {
            // Ключевой запрос: начало координат кисти в системе координат камеры головы.
            ts = tf_buffer_->lookupTransform(camera_frame_, wrist_frame(),
                                             tf2::TimePointZero,
                                             rclcpp::Duration(0, 50'000'000));
        } catch (const tf2::ExtrapolationException &) {
            return;  // joint_states ещё не пришли — молча пропускаем цикл
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "TF %s -> %s недоступен: %s",
                                 camera_frame_.c_str(), wrist_frame().c_str(), ex.what());
            return;
        }

        // Позиция запястья в рамке камеры (оптическая система: x вправо, y вниз, z вперёд).
        const auto &t = ts.transform.translation;
        const double X = t.x, Y = t.y, Z = t.z;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = ts.header.stamp;
        pose.header.frame_id = camera_frame_;
        pose.pose.position.x = X;
        pose.pose.position.y = Y;
        pose.pose.position.z = Z;
        pose.pose.orientation = ts.transform.rotation;
        target_pub_->publish(pose);

        // Frustum test: попадает ли точка в пирамиду видимости камеры.
        // Углы обзора берём из интринсиков: hfov = 2*atan(cx/fx), vfov = 2*atan(cy/fy).
        bool visible = (Z > z_near_);
        if (visible && got_intrinsics_) {
            visible = std::abs(X) <= Z * (cx_ / fx_) &&
                      std::abs(Y) <= Z * (cy_ / fy_);
        }
        std_msgs::msg::Bool vmsg;
        vmsg.data = visible;
        visible_pub_->publish(vmsg);

        if (visible && got_intrinsics_ && Z > z_near_) {
            const double u = fx_ * X / Z + cx_;
            const double v = fy_ * Y / Z + cy_;
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                "wrist(%s) в кадре: X=%.2f Y=%.2f Z=%.2f -> px(%.0f, %.0f)",
                hand_.c_str(), X, Y, Z, u, v);
        }
    }

    // параметры/состояние
    std::string camera_frame_, hand_;
    double rate_hz_{50.0}, z_near_{0.15};
    bool got_intrinsics_{false};
    double fx_{0}, fy_{0}, cx_{0}, cy_{0};

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visible_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_hand_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HandTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
