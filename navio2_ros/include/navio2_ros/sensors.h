#ifndef NAVIO2_ROS_SENSORS_H
#define NAVIO2_ROS_SENSORS_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "navio2_ros/imu.h"
#include "navio2_ros/type_adapters/imu_msg.h"

class NavioSensors : public rclcpp::Node {
    using ImuMsgAdapter = rclcpp::TypeAdapter<ImuData, sensor_msgs::msg::Imu>;
public:
    NavioSensors();

    ~NavioSensors() override = default;

private:
    Navio2::Imu imu_mpu_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ImuMsgAdapter>::SharedPtr publisher_imu_mpu_;

    void timer_callback();

    void publish_imu_data(Navio2::InertialSensorType type, const ImuData& data);
};
#endif //NAVIO2_ROS_SENSORS_H
