#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp> // Для координат цели с камеры
#include <control_msgs/msg/joint_jog.hpp> // Для управления скоростью суставов
#include <algorithm>
#include <cmath>

class RobotCoordinator : public rclcpp::Node {
public:
    RobotCoordinator() : Node("robot_coordinator") {
        // 1. Инициализация издателей (мы отдаем команды)
        head_cmd_pub_ = this->create_publisher<control_msgs::msg::JointJog>("/head_controller/jog_commands", 10);
        hand_cmd_pub_ = this->create_publisher<control_msgs::msg::JointJog>("/hand_controller/jog_commands", 10);

        // 2. Инициализация подписчиков (мы получаем ощущения)
        wrist_force_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/sensors/wrist_force", 10, std::bind(&RobotCoordinator::wrist_force_callback, this, std::placeholders::_1));
        
        hand_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/sensors/hand_state", 10, std::bind(&RobotCoordinator::hand_state_callback, this, std::placeholders::_1));

        vision_target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/vision/target_pixel_coords", 10, std::bind(&RobotCoordinator::vision_target_callback, this, std::placeholders::_1));

        // 3. Главный цикл управления (например, 50 Гц = каждые 20 мс)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&RobotCoordinator::control_loop, this));

        // Параметры (в реальности их лучше загружать из YAML)
        max_grip_force_ = 15.0; // Ньютоны
        image_center_x_ = 320.0; // Центр камеры (для разрешения 640x480)
        
        RCLCPP_INFO(this->get_logger(), "Robot Coordinator initialized. Waiting for sensor data...");
    }

private:
    // --- Обратные вызовы (Сенсоры) ---
    void wrist_force_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        current_wrist_force_ = std::sqrt(msg->x*msg->x + msg->y*msg->y + msg->z*msg->z); // Модуль вектора силы
    }

    void hand_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Предполагаем, что первый сустав - это основной привод захвата
        if (!msg->name.empty() && msg->name[0] == "grip_joint") {
            current_grip_position_ = msg->position[0];
            current_grip_effort_ = msg->effort[0]; // Сила/момент в самом пальце
        }
    }

    void vision_target_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        target_pixel_x_ = msg->x;
        target_pixel_y_ = msg->y;
        target_detected_ = true;
    }

    // --- Главный цикл управления (Мозг) ---
    void control_loop() {
        // ЗАДАЧА 1: Активное зрение (Visual Servoing)
        control_msgs::msg::JointJog head_cmd;
        head_cmd.header.stamp = this->now();
        head_cmd.joint_names = {"neck_pan_joint", "neck_tilt_joint"};
        
        if (target_detected_) {
            // Пропорциональный регулятор (P-регулятор) для наведения
            double error_x = target_pixel_x_ - image_center_x_;
            double error_y = target_pixel_y_ - 240.0; // Центр по Y
            
            // Коэффициент усиления (подбирается экспериментально)
            double k_p_pan = 0.005; 
            double k_p_tilt = 0.005;

            head_cmd.velocities = {
                std::clamp(error_x * k_p_pan, -1.0, 1.0), // Ограничение скорости
                std::clamp(error_y * k_p_tilt, -1.0, 1.0)
            };
        } else {
            head_cmd.velocities = {0.0, 0.0}; // Цель не видна, не двигаемся
        }
        head_cmd_pub_->publish(head_cmd);

        // ЗАДАЧА 2: Управление захватом с ограничением по силе (Force-limited Position Control)
        control_msgs::msg::JointJog hand_cmd;
        hand_cmd.header.stamp = this->now();
        hand_cmd.joint_names = {"grip_joint"};

        if (is_grasping_active_) {
            // Проверяем обратную связь по силе
            if (current_wrist_force_ > max_grip_force_ || std::abs(current_grip_effort_) > max_grip_force_) {
                // ПРЕВЫШЕНИЕ СИЛЫ! Останавливаем движение или слегка разжимаем (импеданс)
                hand_cmd.velocities = {-0.05}; // Микро-разжатие для снятия напряжения
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Force limit reached! Relieving pressure.");
            } else {
                // Сила в норме, продолжаем смыкать пальцы с заданной скоростью
                hand_cmd.velocities = {grip_speed_};
            }
        } else {
            // Режим ожидания или разжатия
            hand_cmd.velocities = {0.0};
        }
        hand_cmd_pub_->publish(hand_cmd);
    }

    // --- Сервисы или методы для смены состояния (вызываются извне) ---
    void start_grasp(double speed) {
        is_grasping_active_ = true;
        grip_speed_ = std::clamp(speed, 0.01, 0.5); // Ограничение скорости захвата
        RCLCPP_INFO(this->get_logger(), "Grasp initiated with speed: %.2f", grip_speed_);
    }

    void stop_grasp() {
        is_grasping_active_ = false;
        RCLCPP_INFO(this->get_logger(), "Grasp stopped.");
    }

    // --- Переменные состояния (Память робота) ---
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr head_cmd_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr hand_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wrist_force_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr hand_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr vision_target_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    double current_wrist_force_ = 0.0;
    double current_grip_position_ = 0.0;
    double current_grip_effort_ = 0.0;
    double target_pixel_x_ = 0.0;
    double target_pixel_y_ = 0.0;
    bool target_detected_ = false;
    
    bool is_grasping_active_ = false;
    double grip_speed_ = 0.1;
    double max_grip_force_ = 15.0;
    double image_center_x_ = 320.0;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotCoordinator>());
    rclcpp::shutdown();
    return 0;
}