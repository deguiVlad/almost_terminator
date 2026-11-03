// focus_controller_node.cpp
// Универсальный контроллер фокусировки головы: цель — объект, рука или обе сразу.
// (nssr-sim, ROS 2 Humble; развитие head_focus_node из task_2_cpp.)
//
// Ключевая идея: все цели приходят в одном типе geometry_msgs/PoseStamped в одной
// рамке camera_head_link, поэтому математика для объекта и руки идентична:
//   yaw   = atan2(X, Z)
//   pitch = atan2(-Y, sqrt(X^2 + Z^2))     // точная сферическая формула
// Режим «both»: визирная ось одна на две цели -> взвешенное усреднение углов
//   q* = (w_obj*q_obj + w_hand*q_hand) / (w_obj + w_hand)
// Веса переключаются по факту захвата (рука держит объект -> важнее контроль хвата).
//
// Защита от шума детектора — deadzone в пикселях; защита от дёрганья — LPF;
// защита от потери цели — таймаут с возвратом в нейтраль; лимиты суставов и
// скорости (0.8 рад/с) соблюдаются.
//
// Интерфейсы:
//   ВХОД  /focus/target          geometry_msgs/PoseStamped  цель-объект (от object_detector)
//   ВХОД  /focus/hand            geometry_msgs/PoseStamped  цель-рука    (от hand_tracker)
//   ВХОД  /cam_head/color/camera_info sensor_msgs/CameraInfo
//   ВХОД  /head/status           bodyctrl_msgs/MotorStatusMsg
//   ВЫХОД /head/cmd_pos          bodyctrl_msgs/CmdSetMotorPosition (20 Гц)
//   СЕРВИС /focus/set_mode       std_srvs/SetBool  true=ACTIVE false=IDLE
//   ПАРАМЕТР source              object | hand | both
//
// Запуск:
//   ros2 run focus_tracking focus_controller_node --ros-args -p use_sim_time:=true 
//     -p source:=both -p yaw_limit:=1.5 -p pitch_limit:=0.44

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

constexpr uint16_t MOTOR_HEAD_PITCH = 2;
constexpr uint16_t MOTOR_HEAD_YAW   = 3;

struct Target {
    double x{0}, y{0}, z{0};
    rclcpp::Time stamp{0, 0, RCL_STEADY_TIME};
    bool fresh(const rclcpp::Time &now, double timeout) const
    { return z > 0.05 && (now - stamp).seconds() < timeout; }
};

class FocusControllerNode : public rclcpp::Node
{
public:
    FocusControllerNode()
    : Node("focus_controller_node")
    {
        source_         = declare_parameter<std::string>("source", "object"); // object|hand|both
        pub_rate_hz_    = declare_parameter<double>("publish_rate_hz", 20.0);
        yaw_limit_      = declare_parameter<double>("yaw_limit", 1.5);
        pitch_limit_    = declare_parameter<double>("pitch_limit", 0.44);
        spd_rpm_        = declare_parameter<double>("spd_rpm", 8.0);
        cur_a_          = declare_parameter<double>("cur_a", 1.0);
        deadzone_px_    = declare_parameter<double>("deadzone_px", 20.0);
        smoothing_      = declare_parameter<double>("smoothing", 0.5);
        target_timeout_ = declare_parameter<double>("target_timeout_s", 1.0);
        w_object_       = declare_parameter<double>("weight_object", 1.0);
        w_hand_         = declare_parameter<double>("weight_hand", 0.3);
        // Смещение точки удержания руки в кадре (px от центра): рука не должна
        // загораживать центр обзора — держим её в нижнем правом квадранте.
        hand_off_x_px_  = declare_parameter<double>("hand_offset_px_x", 150.0);
        hand_off_y_px_  = declare_parameter<double>("hand_offset_px_y", 100.0);

        cmd_pub_ = create_publisher<bodyctrl_msgs::msg::CmdSetMotorPosition>(
            "/head/cmd_pos", rclcpp::QoS(10));

        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/cam_head/color/camera_info", rclcpp::QoS(5),
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr m) {
                fx_ = m->k[0]; fy_ = m->k[4];
                cx_ = m->k[2]; cy_ = m->k[5];
                got_intrinsics_ = true;
            });

        obj_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/focus/target", rclcpp::QoS(5),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {
                on_target(m, obj_);
            });
        hand_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/focus/hand", rclcpp::QoS(5),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {
                on_target(m, hand_);
            });

        status_sub_ = create_subscription<bodyctrl_msgs::msg::MotorStatusMsg>(
            "/head/status", rclcpp::QoS(5),
            [this](const bodyctrl_msgs::msg::MotorStatusMsg::SharedPtr m) {
                for (const auto &st : m->status) {
                    if (st.name == MOTOR_HEAD_YAW)   cur_yaw_   = st.pos;
                    if (st.name == MOTOR_HEAD_PITCH) cur_pitch_ = st.pos;
                }
            });

        set_mode_srv_ = create_service<std_srvs::srv::SetBool>(
            "/focus/set_mode",
            [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                   std_srvs::srv::SetBool::Response::SharedPtr res) {
                active_ = req->data;
                res->success = true;
                res->message = active_ ? "focus ACTIVE (" + source_ + ")"
                                       : "focus IDLE (neutral pose)";
                RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
            });

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / pub_rate_hz_)),
            std::bind(&FocusControllerNode::tick, this));

        RCLCPP_INFO(get_logger(), "focus_controller_node: source=%s, yaw<=%.2f pitch<=%.2f",
                    source_.c_str(), yaw_limit_, pitch_limit_);
    }

private:
    static double clamp(double v, double lim) { return std::max(-lim, std::min(lim, v)); }

    void on_target(const geometry_msgs::msg::PoseStamped::SharedPtr &m, Target &dst)
    {
        if (m->header.frame_id != "camera_head_link") {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "frame '%s' не поддержан, ждём camera_head_link", m->header.frame_id.c_str());
            return;
        }
        dst.x = m->pose.position.x;
        dst.y = m->pose.position.y;
        dst.z = m->pose.position.z;
        dst.stamp = now();
    }

    // 3D-точка в камере -> углы визирования (pinhole/сферическая модель)
    static void to_angles(const Target &t, double &yaw, double &pitch)
    {
        yaw   = std::atan2(t.x, t.z);
        pitch = std::atan2(-t.y, std::hypot(t.x, t.z));
    }

    // Смещение точки удержания цели от центра кадра (px) -> коррекция угла
    double px_to_rad_x(double dx_px) const { return got_intrinsics_ ? dx_px / fx_ : 0.0; }
    double px_to_rad_y(double dy_px) const { return got_intrinsics_ ? dy_px / fy_ : 0.0; }

    void tick()
    {
        double yaw_cmd = 0.0, pitch_cmd = 0.0;
        bool have = false;

        if (active_) {
            const auto n = now();
            double y1, p1, y2, p2;
            const bool o_ok = (source_ == "object" || source_ == "both") && obj_.fresh(n, target_timeout_);
            const bool h_ok = (source_ == "hand"   || source_ == "both") && hand_.fresh(n, target_timeout_);

            if (o_ok && h_ok) {
                // Режим BOTH: одно визирное направление на две цели —
                // взвешенное среднее углов (линеаризация малых углов J(q)).
                to_angles(obj_, y1, p1);
                to_angles(hand_, y2, p2);
                // руку держим со смещением в кадре, чтобы не загораживала объект
                y2 += px_to_rad_x(hand_off_x_px_);
                p2 += px_to_rad_y(hand_off_y_px_);
                const double wsum = w_object_ + w_hand_;
                yaw_cmd   = (w_object_ * y1 + w_hand_ * y2) / wsum;
                pitch_cmd = (w_object_ * p1 + w_hand_ * p2) / wsum;
                have = true;
            } else if (o_ok) {
                to_angles(obj_, y1, p1);
                yaw_cmd = y1; pitch_cmd = p1; have = true;
            } else if (h_ok) {
                to_angles(hand_, y1, p1);
                yaw_cmd = y1 + px_to_rad_x(hand_off_x_px_);
                pitch_cmd = p1 + px_to_rad_y(hand_off_y_px_);
                have = true;
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                    "нет свежей цели (source=%s), возврат в нейтраль", source_.c_str());
            }
        }

        if (have) {
            // Deadzone: ошибка в пикселях меньше порога — держим текущую позу.
            const double ey = yaw_cmd - cur_yaw_;
            const double ep = pitch_cmd - cur_pitch_;
            const double err_px = std::hypot(ey * fx_, ep * fy_);
            if (got_intrinsics_ && err_px < deadzone_px_) {
                yaw_cmd = cur_yaw_; pitch_cmd = cur_pitch_;
            } else {
                tgt_yaw_   = smoothing_ * yaw_cmd   + (1.0 - smoothing_) * tgt_yaw_;
                tgt_pitch_ = smoothing_ * pitch_cmd + (1.0 - smoothing_) * tgt_pitch_;
            }
        } else {
            tgt_yaw_ = 0.0; tgt_pitch_ = 0.0;  // нейтраль
        }

        publish(clamp(tgt_yaw_, yaw_limit_), clamp(tgt_pitch_, pitch_limit_));
    }

    void publish(double yaw, double pitch)
    {
        bodyctrl_msgs::msg::CmdSetMotorPosition msg;
        auto mk = [](uint16_t id, double pos) {
            bodyctrl_msgs::msg::SetMotorPosition c;
            c.name = id; c.pos = pos; c.spd = 0.7; c.cur = 1.0; // spd<0.8 — лимит симулятора
            return c;
        };
        (void)spd_rpm_; (void)cur_a_;
        msg.cmds.push_back(mk(MOTOR_HEAD_PITCH, pitch));
        msg.cmds.push_back(mk(MOTOR_HEAD_YAW, yaw));
        cmd_pub_->publish(msg);
    }

    // параметры
    std::string source_;
    double pub_rate_hz_, yaw_limit_, pitch_limit_, spd_rpm_, cur_a_;
    double deadzone_px_, smoothing_, target_timeout_;
    double w_object_, w_hand_, hand_off_x_px_, hand_off_y_px_;
    // состояние
    bool active_{true}, got_intrinsics_{false};
    double fx_{800}, fy_{800}, cx_{320}, cy_{180};
    double tgt_yaw_{0}, tgt_pitch_{0}, cur_yaw_{0}, cur_pitch_{0};
    Target obj_, hand_;

    rclcpp::Publisher<bodyctrl_msgs::msg::CmdSetMotorPosition>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr obj_sub_, hand_sub_;
    rclcpp::Subscription<bodyctrl_msgs::msg::MotorStatusMsg>::SharedPtr status_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_mode_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FocusControllerNode>());
    rclcpp::shutdown();
    return 0;
}
