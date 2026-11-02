// head_demo.cpp
// Паблишер демонстрации КРУГОВЫХ движений головой робота (nssr-sim, ROS2 Humble).
//
// Публикует позиционные команды в /head/cmd_pos (bodyctrl_msgs/msg/CmdSetMotorPosition).
//
// Голова (см. чит-шит интерфейсов хакатона):
//   motor 1 = head_roll_joint  (наклон)
//   motor 2 = head_pitch_joint (кивок)
//   motor 3 = head_yaw_joint   (поворот)
//
// Траектория — «окружность» в плоскости кивок/поворот: pitch и yaw задаются
// двумя синусоидами, сдвинутыми по фазе на 90° (квадратура). Наклон головы
// (roll, motor 1) НЕ используется — круг только «кивок + поворот».
// В каждом кадре одновременно движутся ОБА мотора (движение суммарно
// несколькими моторами).
//
// Плавность: фаза траектории вычисляется из ВРЕМЕНИ СИМУЛЯТОРА — узел подписан
// на топик /clock (rosgraph_msgs/msg/Clock, sim time, ~20 Гц по чит-шиту) и
// сам тикает по часам симулятора (use_sim_time), поэтому движение не
// «дёргается», даже если реальный темп симуляции отличается от wall-clock.
//
// Запуск:
//   ros2 run task_2_cpp head_demo_node --ros-args -p circle_period_s:=8.0

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "bodyctrl_msgs/msg/cmd_set_motor_position.hpp"
#include "bodyctrl_msgs/msg/set_motor_position.hpp"
#include "rosgraph_msgs/msg/clock.hpp"



// ID моторов головы (первая цифра 0 = голова)
constexpr uint16_t MOTOR_HEAD_PITCH = 2;
constexpr uint16_t MOTOR_HEAD_YAW = 3;

class HeadCirclePublisher : public rclcpp::Node
{
public:
    HeadCirclePublisher()
    : Node("head_circle_publisher")
    {
        // Узел работает по времени симулятора (топик /clock)
        set_parameter(rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

        // Параметры сценария
        period_s_  = declare_parameter<double>("circle_period_s", 8.0);  // оборот окружности, c
        amp_pitch_ = declare_parameter<double>("amp_pitch", 0.15);       // амплитуда pitch (кивок), рад
        amp_yaw_   = declare_parameter<double>("amp_yaw", 0.25);         // амплитуда yaw (поворот), рад
        spd_rpm_   = declare_parameter<double>("spd_rpm", 5.0);          // скорость моторов, rpm
        cur_a_     = declare_parameter<double>("cur_a", 1.0);            // ограничение тока, A
        const double rate_hz = declare_parameter<double>("publish_rate_hz", 20.0);

        cmd_pub_ = create_publisher<bodyctrl_msgs::msg::CmdSetMotorPosition>(
            "/head/cmd_pos", rclcpp::QoS(10));

        // Подписка на часы симулятора: берём время напрямую из /clock
        // (даже если use_sim_time будет переопределён снаружи)
        clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliability(rclcpp::ReliabilityPolicy::BestEffort),
            [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
                if (!got_clock_) {
                    t0_ = rclcpp::Time(msg->clock);   // старт отсчёта фазы
                }
                last_sim_time_ = rclcpp::Time(msg->clock);
                got_clock_ = true;
            });

        // Тикаем по часам симулятора (таймер узла использует get_clock() при use_sim_time)
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / rate_hz)),
            std::bind(&HeadCirclePublisher::tick, this));

        RCLCPP_INFO(get_logger(),
            "Head circle demo started (no roll): /head/cmd_pos, period %.1f s, "
            "amp(pitch=%.2f, yaw=%.2f rad), rate %.1f Hz (sim time from /clock)",
            period_s_, amp_pitch_, amp_yaw_, rate_hz);
    }

private:
    static bodyctrl_msgs::msg::SetMotorPosition make_cmd(uint16_t id, float pos, float spd, float cur)
    {
        bodyctrl_msgs::msg::SetMotorPosition cmd;
        cmd.name = id;
        cmd.pos = pos;   // целевой угол, рад
        cmd.spd = spd;   // скорость, rpm
        cmd.cur = cur;   // ток, A
        return cmd;
    }

    void tick()
    {
        // Ждём первые тики времени симулятора, чтобы не «прыгать» до /clock
        if (!got_clock_) {
            return;
        }

        // Время симулятора: собственные часы узла (use_sim_time -> /clock),
        // fallback — последнее сохранённое сообщение /clock
        double now_s = last_sim_time_.seconds();
        try {
            const rclcpp::Time t = get_clock()->now();
            if (t.seconds() >= now_s) {
                now_s = t.seconds();
            }
        } catch (const std::exception &) {
            // часы ещё не готовы — используем последний /clock
        }

        const double t = now_s - t0_.seconds();
        const double w = 2.0 * M_PI / period_s_;   // угловая частота круга

        // Окружность БЕЗ НАКЛОНА: pitch (кивок) и yaw (поворот) в квадратуре
        // (сдвиг фаз 90°). Оба мотора движутся одновременно -> суммарный вектор
        // взгляда плавно чертит круг в плоскости вверх-вниз / влево-вправо.
        const float pitch = static_cast<float>(amp_pitch_ * std::sin(w * t));
        const float yaw   = static_cast<float>(amp_yaw_   * std::cos(w * t));

        const float spd = static_cast<float>(spd_rpm_);
        const float cur = static_cast<float>(cur_a_);

        bodyctrl_msgs::msg::CmdSetMotorPosition msg;
        msg.cmds = {
            make_cmd(MOTOR_HEAD_PITCH, pitch, spd, cur),
            make_cmd(MOTOR_HEAD_YAW, yaw, spd, cur),
        };
        cmd_pub_->publish(msg);

        RCLCPP_DEBUG(get_logger(), "sim t=%.2f s -> pitch=%.3f yaw=%.3f rad",
                     t, pitch, yaw);
    }

    rclcpp::Publisher<bodyctrl_msgs::msg::CmdSetMotorPosition>::SharedPtr cmd_pub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool got_clock_{false};
    rclcpp::Time last_sim_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time t0_{0, 0, RCL_ROS_TIME};

    double period_s_{8.0};
    double amp_pitch_{0.15};
    double amp_yaw_{0.25};
    double spd_rpm_{5.0};
    double cur_a_{1.0};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadCirclePublisher>());
    rclcpp::shutdown();
    return 0;
}
