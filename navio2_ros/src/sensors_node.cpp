#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class NavioSensors : public rclcpp::Node {
public:
    NavioSensors()
            : Node("navio_sensors"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("sensors", rclcpp::QoS(10).best_effort());
        timer_ = this->create_wall_timer(
                500ms, [this] { timer_callback(); });
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

    void timer_callback() {
        // check for any subscriber
        if (publisher_->get_subscription_count() == 0) {
            RCLCPP_DEBUG(this->get_logger(), "No subscribers, not publishing");
            return;
        }

        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavioSensors>());
    rclcpp::shutdown();
    return 0;
}
