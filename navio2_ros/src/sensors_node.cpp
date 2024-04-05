#include "navio2_ros/sensors.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavioSensors>());
    rclcpp::shutdown();
    return 0;
}
