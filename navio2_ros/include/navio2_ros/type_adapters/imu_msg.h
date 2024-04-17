#ifndef NAVIO2_ROS_IMU_MSG_H
#define NAVIO2_ROS_IMU_MSG_H

#include <rclcpp/type_adapter.hpp>

#include <sensor_msgs/msg/imu.hpp>

struct ImuData {
    rclcpp::Time timestamp;
    std::string frame_id;
    std::array<double, 3> linear_acceleration;
    std::array<double, 3> angular_velocity;
};

template<>
struct rclcpp::TypeAdapter<ImuData, sensor_msgs::msg::Imu> {
    using is_specialized = std::true_type;
    using custom_type = ImuData;
    using ros_message_type = sensor_msgs::msg::Imu;

    static void convert_to_ros_message(const ImuData &from, sensor_msgs::msg::Imu &to) {
        to.header.stamp = from.timestamp;
        to.linear_acceleration.x = from.linear_acceleration[0];
        to.linear_acceleration.y = from.linear_acceleration[1];
        to.linear_acceleration.z = from.linear_acceleration[2];
        to.angular_velocity.x = from.angular_velocity[0];
        to.angular_velocity.y = from.angular_velocity[1];
        to.angular_velocity.z = from.angular_velocity[2];
    }

    static void convert_to_std(const sensor_msgs::msg::Imu &from, ImuData &to) {
        to.linear_acceleration = {from.linear_acceleration.x, from.linear_acceleration.y, from.linear_acceleration.z};
        to.angular_velocity = {from.angular_velocity.x, from.angular_velocity.y, from.angular_velocity.z};
    }
};

#endif //NAVIO2_ROS_IMU_MSG_H
