// head_demo.cpp
// Паблишер для демонстрации движения головой робота.
//
// Публикует позиционные команды в топик /head/cmd_pos
// (тип bodyctrl_msgs/msg/CmdSetMotorPosition).
//
// Голова имеет 3 степени свободы (см. чит-шит интерфейсов):
//   motor 1 = head_roll_joint
//   motor 2 = head_pitch_joint
//   motor 3 = head_yaw_joint
//
// Узел поочерёдно выполняет "демо-сценарий": кивок (pitch),
// покачивание (roll) и повороты (yaw), затем возвращается в нейтраль.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "bodyctrl_msgs/msg/cmd_set_motor_position.hpp"

using namespace std::chrono_literals;

// ID моторов головы (MotorName: 00 = head, моторы 1..3)
constexpr uint16_t MOTOR_HEAD_ROLL = 1;
constexpr uint16_t MOTOR_HEAD_PITCH = 2;
constexpr uint16_t MOTOR_HEAD_YAW = 3;

class HeadDemoPublisher : public rclcpp::Node
{
public:
    HeadDemoPublisher()
    : Node("head_demo_publisher"), index_(0)
    {
        // Параметры для тонкой настройки сцены
        period_ms_ = declare_parameter<int>("period_ms", 500);
        spd_rpm_ = declare_parameter<double>("spd_rpm", 10.0);
        cur_a_ = declare_parameter<double>("cur_a", 1.0);

        cmd_pub_ = create_publisher<bodyctrl_msgs::msg::CmdSetMotorPosition>(
            "/head/cmd_pos", rclcpp::QoS(10));

        // Сценарий: список кадров, каждый кадр — набор команд для моторов головы
        auto make_cmd = [this](uint16_t name, float pos) {
            bodyctrl_msgs::msg::SetMotorPosition cmd;
            cmd.name = name;
            cmd.pos = pos;          // целевой угол, рад
            cmd.spd = static_cast<float>(spd_rpm_);  // скорость, rpm
            cmd.cur = static_cast<float>(cur_a_);    // ограничение тока, A
            return cmd;
        };

        // Кивок вниз-вверх (pitch), наклон (roll), повороты (yaw)
        frames_ = {
            /* 0 */ {make_cmd(MOTOR_HEAD_PITCH, 0.0f),  make_cmd(MOTOR_HEAD_ROLL, 0.0f),  make_cmd(MOTOR_HEAD_YAW, 0.0f)},   // нейтраль
            /* 1 */ {make_cmd(MOTOR_HEAD_PITCH, 0.2f),  make_cmd(MOTOR_HEAD_ROLL, 0.0f),  make_cmd(MOTOR_HEAD_YAW, 0.0f)},   // кивок вниз
            /* 2 */ {make_cmd(MOTOR_HEAD_PITCH, -0.1f), make_cmd(MOTOR_HEAD_ROLL, 0.0f),  make_cmd(MOTOR_HEAD_YAW, 0.0f)},   // кивок вверх
            /* 3 */ {make_cmd(MOTOR_HEAD_PITCH, 0.0f),  make_cmd(MOTOR_HEAD_ROLL, 0.15f), make_cmd(MOTOR_HEAD_YAW, 0.0f)},   // наклон вбок
            /* 4 */ {make_cmd(MOTOR_HEAD_PITCH, 0.0f),  make_cmd(MOTOR_HEAD_ROLL, -0.15f),make_cmd(MOTOR_HEAD_YAW, 0.0f)},   // наклон в другую сторону
            /* 5 */ {make_cmd(MOTOR_HEAD_PITCH, 0.0f),  make_cmd(MOTOR_HEAD_ROLL, 0.0f),  make_cmd(MOTOR_HEAD_YAW, 0.3f)},   // поворот вправо
            /* 6 */ {make_cmd(MOTOR_HEAD_PITCH, 0.0f),  make_cmd(MOTOR_HEAD_ROLL, 0.0f),  make_cmd(MOTOR_HEAD_YAW, -0.3f)},  // поворот влево
            /* 7 */ {make_cmd(MOTOR_HEAD_PITCH, 0.0f),  make_cmd(MOTOR_HEAD_ROLL, 0.0f),  make_cmd(MOTOR_HEAD_YAW, 0.0f)},   // обратно в нейтраль
        };

        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms_),
            std::bind(&HeadDemoPublisher::publish_frame, this));

        RCLCPP_INFO(get_logger(),
            "Head demo publisher started: /head/cmd_pos, %zu frames, period %d ms",
            frames_.size(), period_ms_);
    }

private:
    void publish_frame()
    {
        if (cmd_pub_->get_subscription_count() == 0u) {
            // Никто не слушает — ничего не публикуем (экономим ресурсы)
            return;
        }

        const auto & frame = frames_[index_];
        for (const auto & c : frame) {
            RCLCPP_DEBUG(get_logger(), "head motor %u -> pos %.2f rad (spd %.1f rpm, cur %.2f A)",
                         c.name, c.pos, c.spd, c.cur);
        }

        bodyctrl_msgs::msg::CmdSetMotorPosition msg;
        msg.cmds = frame;
        cmd_pub_->publish(msg);

        RCLCPP_INFO(get_logger(), "Frame %zu/%zu published (roll/pitch/yaw targets sent)",
                    index_ + 1, frames_.size());

        index_ = (index_ + 1) % frames_.size();
    }

    rclcpp::Publisher<bodyctrl_msgs::msg::CmdSetMotorPosition>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::vector<bodyctrl_msgs::msg::SetMotorPosition>> frames_;
    size_t index_;

    int period_ms_{500};
    double spd_rpm_{10.0};
    double cur_a_{1.0};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadDemoPublisher>());
    rclcpp::shutdown();
    return 0;
}
