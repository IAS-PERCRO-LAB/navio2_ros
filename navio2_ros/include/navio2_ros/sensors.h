#ifndef NAVIO2_ROS_SENSORS_H
#define NAVIO2_ROS_SENSORS_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class NavioSensors : public rclcpp::Node {
public:
    NavioSensors();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

    void timer_callback();
};
#endif //NAVIO2_ROS_SENSORS_H
