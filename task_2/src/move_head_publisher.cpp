// Demo head movement publisher for nssr-polygon hackathon
#include <rclcpp/rclcpp.hpp>
#include <bodyctrl_msgs/msg/cmd_set_motor_position.hpp>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>

using bodyctrl_msgs::msg::CmdSetMotorPosition;

class DemoHeadPublisher : public rclcpp::Node {
public:
  DemoHeadPublisher() : Node("demo_head_publisher") {
    // Publish to /head/cmd_pos topic with head_yaw_joint (motor id 3)
    publisher_ = create_publisher<CmdSetMotorPosition>("/head/cmd_pos", 10);
    
    RCLCPP_INFO(get_logger(), "DemoHeadPublisher initialized");
    RCLCPP_INFO(get_logger(), "Publishing to: /head/cmd_pos");
    RCLCPP_INFO(get_logger(), "Motor: head_yaw_joint (id=3)");
  }

private:
  rclcpp::Publisher<CmdSetMotorPosition>::SharedPtr publisher_;

  void publishHeadYawMove(double angle, double speed) {
    auto cmd_msg = std::make_shared<CmdSetMotorPosition>();
    
    // Set header
    cmd_msg->header.stamp = this->now();
    cmd_msg->header.frame_id = "head_link";
    
    // Configure command for head_yaw_joint (motor id 3)
    cmd_msg->cmds.resize(1);
    cmd_msg->cmds[0].name = 3;  // head_yaw_joint
    cmd_msg->cmds[0].pos = angle;      // position in radians
    cmd_msg->cmds[0].spd = speed;      // speed in rpm
    cmd_msg->cmds[0].cur = 1.5;        // current (approximate for demo)
    
    publisher_->publish(cmd_msg);
    RCLCPP_DEBUG(get_logger(), "Published: pos=%.2f rad, spd=%.0f rpm", angle, speed);
  }

  void headYawSineWaveContinuous() {
    RCLCPP_INFO(get_logger(), "Starting continuous sine wave demo...");
    
    double amplitude = M_PI / 2; // ~90 degrees
    double frequency_hz = 1.5;
    
    while (this->get_clock()->is_ok()) {
      double t = this->now().nanoseconds() * 1e-9;
      
      publishHeadYawMove(
        amplitude * sin(frequency_hz * t), 
        30.0  // rpm
      );
      
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto spawner = std::make_shared<DemoHeadPublisher>();
  
  RCLCPP_INFO(spawner->get_logger(), "Demo head movement publisher started");
  spawner->headYawSineWaveContinuous();
  
  rclcpp::shutdown();
  return 0;
}
