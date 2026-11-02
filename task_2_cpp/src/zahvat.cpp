// hand_controller.cpp
// Узел управления кистью робота (на примере левой руки)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>

class HandController : public rclcpp::Node
{
public:
    HandController() : Node("hand_controller"), is_open_(false)
    {
        // 1. Publisher для управления кистью.
        // Топик из шпаргалки: /inspire_hand/ctrl/left_hand
        ctrl_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/inspire_hand/ctrl/left_hand", 10);

        // 2. Subscriber для чтения состояния кисти.
        // Топик из шпаргалки: /inspire_hand/state/left_hand
        state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/inspire_hand/state/left_hand", 10,
            std::bind(&HandController::state_callback, this, std::placeholders::_1));

        // 3. Таймер: каждые 2 секунды будем менять состояние кисти (открыть/закрыть)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&HandController::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Hand controller node started");
    }

private:
    // Колбэк: вызывается, когда симулятор отправляет текущее состояние кисти (20 Hz)
    void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // В шпаргалке сказано: names 1-6, position 0-1.
        // Просто выведем текущее положение большого пальца (index 5, имя "6")
        if (msg->position.size() >= 6) {
            RCLCPP_INFO(this->get_logger(), "Current thumb position: %.2f", msg->position[5]);
        }
    }

    // Колбэк таймера: отправляем команду на кисть
    void timer_callback()
    {
        auto msg = sensor_msgs::msg::JointState();
        
        // Заполняем имена пальцев согласно шпаргалке
        msg.name = {"1", "2", "3", "4", "5", "6"};
        
        // Заполняем позиции. 0 = закрыто, 1 = открыто.
        double target_pos = is_open_ ? 1.0 : 0.0;
        
        // Открываем/закрываем все 6 пальцев одновременно
        msg.position = {target_pos, target_pos, target_pos, target_pos, target_pos, target_pos};
        
        // Отправляем команду в симулятор
        ctrl_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "Sending command: %s (pos: %.1f)", 
                    is_open_ ? "OPEN" : "CLOSE", target_pos);
        
        // Меняем флаг для следующего раза
        is_open_ = !is_open_;
    }

    // Поля класса
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ctrl_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool is_open_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HandController>());
    rclcpp::shutdown();
    return 0;
}
