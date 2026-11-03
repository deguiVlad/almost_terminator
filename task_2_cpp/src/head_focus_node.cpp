// head_focus_node.cpp
// Нода поворота головы для фокуса на конкретных объектах (nssr-sim, ROS2 Humble).
//
// Идея: узел получает 2D-координаты цели (центр bounding box объекта на изображении
// головы) и/или её 3D-положение, переводит цель в углы head_pitch / head_yaw и
// отправляет позиционные команды в /head/cmd_pos.
//
// Интерфейсы (см. docs/nssr_hackathon_interfaces_cheatsheet_RU.md):
//   ВХОД  /head_focus/target        geometry_msgs/PoseStamped   (опционально)
//         — позиция цели в frame_id=camera_head_link (оптическая ось = +Z, x вправо, y вниз)
//   ВХОД  /cam_head/color/camera_info sensor_msgs/CameraInfo    (K: fx, fy, cx, cy)
//   ВХОД  /head/status              bodyctrl_msgs/MotorStatusMsg (текущие yaw/pitch)
//   ВЫХОД /head/cmd_pos             bodyctrl_msgs/CmdSetMotorPosition
//         motor 1 = head_roll_joint, 2 = head_pitch_joint (кивок), 3 = head_yaw_joint (поворот)
//
// Формула пиксель -> угол (pinhole):
//   yaw   = atan2(u - cx, fx)     (горизонтальное отклонение от оптической оси)
//   pitch = atan2(v - cy, fy)     (вертикальное отклонение)
// Если известна дистанция d до цели (depth или PoseStamped.position.z), можно
// использовать точечную формулу yaw = atan2(x, z), pitch = atan2(-y, z) — здесь
// реализованы оба пути.
//
// Управление режимом (service /head_focus/set_mode, std_srvs/SetBool):
//   true  = ACTIVE (следим за целью)
//   false = IDLE   (возврат в нейтральную позицию)
//
// Запуск:
//   ros2 run task_2_cpp head_focus_node --ros-args \
//     -p yaw_limit:=0.6 -p pitch_limit:=0.4 -p deadzone_px:=20

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "bodyctrl_msgs/msg/cmd_set_motor_position.hpp"
#include "bodyctrl_msgs/msg/set_motor_position.hpp"
#include "bodyctrl_msgs/msg/motor_status_msg.hpp"

// ID моторов головы (см. шпаргалку: 1 = roll, 2 = pitch, 3 = yaw)
[[maybe_unused]] constexpr uint16_t MOTOR_HEAD_ROLL = 1;
constexpr uint16_t MOTOR_HEAD_PITCH = 2;
constexpr uint16_t MOTOR_HEAD_YAW = 3;

class HeadFocusNode : public rclcpp::Node
{
public:
    HeadFocusNode()
    : Node("head_focus_node")
    {
        // --- параметры -----------------------------------------------------
        period_ms_    = declare_parameter<double>("control_period_ms", 50.0); // 20 Гц
        pub_rate_hz_  = declare_parameter<double>("publish_rate_hz", 20.0);
        yaw_limit_    = declare_parameter<double>("yaw_limit", 0.7);    // рад, лимиты сустава
        pitch_limit_  = declare_parameter<double>("pitch_limit", 0.5);
        roll_target_  = declare_parameter<double>("roll_neutral", 0.0);
        spd_rpm_      = declare_parameter<double>("spd_rpm", 8.0);
        cur_a_        = declare_parameter<double>("cur_a", 1.0);
        deadzone_px_  = declare_parameter<double>("deadzone_px", 20.0); // «в кадре» без движения
        lead_gain_    = declare_parameter<double>("lead_gain", 0.3);    // упреждение по скорости цели
        target_timeout_ = declare_parameter<double>("target_timeout_s", 1.0);
        smoothing_    = declare_parameter<double>("smoothing", 0.5);    // LPF цель = a*new+(1-a)*old

        cmd_pub_ = create_publisher<bodyctrl_msgs::msg::CmdSetMotorPosition>(
            "/head/cmd_pos", rclcpp::QoS(10));

        // camera_info: внутренние параметры камеры (fx, fy, cx, cy)
        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/cam_head/color/camera_info", rclcpp::QoS(5),
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr m) {
                fx_ = m->k[0]; fy_ = m->k[4];
                cx_ = m->k[2]; cy_ = m->k[5];
                got_intrinsics_ = true;
            });

        // Цель как 3D-точка в системе координат камеры
        target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/head_focus/target", rclcpp::QoS(5),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {
                if (m->header.frame_id != "camera_head_link") {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "frame '%s' не поддержан, ждём camera_head_link", m->header.frame_id.c_str());
                    return;
                }
                tx_ = m->pose.position.x; ty_ = m->pose.position.y; tz_ = m->pose.position.z;
                last_target_time_ = now();
                got_target_3d_ = true;
            });

        // Обратная связь: текущие углы головы (для deadzone и упреждения)
        status_sub_ = create_subscription<bodyctrl_msgs::msg::MotorStatusMsg>(
            "/head/status", rclcpp::QoS(5),
            [this](const bodyctrl_msgs::msg::MotorStatusMsg::SharedPtr m) {
                for (const auto& st : m->status) {
                    if (st.name == MOTOR_HEAD_YAW) cur_yaw_ = st.pos;
                    if (st.name == MOTOR_HEAD_PITCH) cur_pitch_ = st.pos;
                }
            });

        // Сервис включения/выключения слежения
        set_mode_srv_ = create_service<std_srvs::srv::SetBool>(
            "/head_focus/set_mode",
            [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                   std_srvs::srv::SetBool::Response::SharedPtr res) {
                active_ = req->data;
                res->success = true;
                res->message = active_ ? "focus ACTIVE" : "focus IDLE (neutral pose)";
                RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
            });

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / pub_rate_hz_)),
            std::bind(&HeadFocusNode::tick, this));

        RCLCPP_INFO(get_logger(),
            "head_focus_node готов. yaw_limit=%.2f рад, pitch_limit=%.2f рад, mode=%s",
            yaw_limit_, pitch_limit_, active_ ? "ACTIVE" : "IDLE");
    }

private:
    static double clamp(double v, double lim) { return std::max(-lim, std::min(lim, v)); }

    void tick()
    {
        // потеря цели -> возврат в нейтраль
        if (active_ && got_target_3d_) {
            const double age = (now() - last_target_time_).seconds();
            lost_target_ = (age > target_timeout_);
            if (lost_target_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                    "цель потеряна (%.1f с), возврат в нейтраль", age);
            }
        }

        double yaw_cmd = 0.0, pitch_cmd = 0.0;

        if (active_ && got_target_3d_ && !lost_target_ && tz_ > 0.05) {
            // Точечная формула: цель задана 3D-координатами камеры (z — вдоль оптической оси)
            const double yaw = std::atan2(tx_, tz_);
            const double pitch = std::atan2(-ty_, tz_);

            // Deadzone: если голова уже смотрит достаточно близко — не дёргаем моторы
            const double err_rad = std::hypot(yaw - cur_yaw_, pitch - cur_pitch_);
            const double err_px = err_rad * ((fx_ + fy_) * 0.25);  // грубая оценка в пикселях
            if (got_intrinsics_ && err_px < deadzone_px_) {
                yaw_cmd = cur_yaw_; pitch_cmd = cur_pitch_;  // удержание текущей позы
            } else {
                // LPF для плавности + ограничение суставов
                tgt_yaw_ = smoothing_ * yaw + (1.0 - smoothing_) * tgt_yaw_;
                tgt_pitch_ = smoothing_ * pitch + (1.0 - smoothing_) * tgt_pitch_;
                yaw_cmd = clamp(tgt_yaw_, yaw_limit_);
                pitch_cmd = clamp(tgt_pitch_, pitch_limit_);
            }
        } else {
            tgt_yaw_ = 0.0; tgt_pitch_ = 0.0;  // нейтраль
        }

        publish(yaw_cmd, pitch_cmd);
    }

    void publish(double yaw, double pitch)
    {
        bodyctrl_msgs::msg::CmdSetMotorPosition msg;
        auto mk = [](uint16_t id, double pos, double spd, double cur) {
            bodyctrl_msgs::msg::SetMotorPosition c;
            c.name = id; c.pos = pos; c.spd = spd; c.cur = cur;
            return c;
        };
        // Командуем pitch и yaw одним кадром (одновременное движение двух моторов)
        msg.cmds.push_back(mk(MOTOR_HEAD_PITCH, pitch, spd_rpm_, cur_a_));
        msg.cmds.push_back(mk(MOTOR_HEAD_YAW, yaw, spd_rpm_, cur_a_));
        cmd_pub_->publish(msg);
    }

    // параметры
    double period_ms_, pub_rate_hz_, yaw_limit_, pitch_limit_, roll_target_;
    double spd_rpm_, cur_a_, deadzone_px_, lead_gain_, target_timeout_, smoothing_;
    // состояние
    bool active_{true}, got_intrinsics_{false}, got_target_3d_{false}, lost_target_{false};
    double fx_{0}, fy_{0}, cx_{0}, cy_{0};
    double tx_{0}, ty_{0}, tz_{0};
    double tgt_yaw_{0}, tgt_pitch_{0}, cur_yaw_{0}, cur_pitch_{0};
    rclcpp::Time last_target_time_{0, 0, RCLCPP_STEADY_TIME};

    rclcpp::Publisher<bodyctrl_msgs::msg::CmdSetMotorPosition>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Subscription<bodyctrl_msgs::msg::MotorStatusMsg>::SharedPtr status_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_mode_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadFocusNode>());
    rclcpp::shutdown();
    return 0;
}
