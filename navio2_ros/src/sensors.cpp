#include "navio2_ros/sensors.h"

#include <chrono>
#include <functional>
#include <string>

using namespace std::chrono_literals;

NavioSensors::NavioSensors()
        : Node("navio_sensors"), imu_mpu_(Navio2::InertialSensorType::MPU9250) {
    // TODO: make the other one too and enable them with a parameter

    // Publishers
    publisher_imu_mpu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/mpu", rclcpp::QoS(10).best_effort());

    // Timers
    timer_ = this->create_wall_timer(
            500ms, [this] { timer_callback(); });
}

void NavioSensors::timer_callback() {
    // check for any subscriber
    if (publisher_imu_mpu_->get_subscription_count() == 0) {
        RCLCPP_DEBUG(this->get_logger(), "No subscribers, not publishing");
        return;
    }

    auto message = sensor_msgs::msg::Imu();
    message.header.stamp = this->now();
    message.header.frame_id = "navio"; // TODO: set the frame id according to the sensor
    imu_mpu_.read_accelerometer(reinterpret_cast<float *>(&message.linear_acceleration.x),
                            reinterpret_cast<float *>(&message.linear_acceleration.y),
                            reinterpret_cast<float *>(&message.linear_acceleration.z));
    imu_mpu_.read_gyroscope(reinterpret_cast<float *>(&message.angular_velocity.x),
                        reinterpret_cast<float *>(&message.angular_velocity.y),
                        reinterpret_cast<float *>(&message.angular_velocity.z));

    // TODO: magnetic field with http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/MagneticField.html

    // TODO: covariances!

    RCLCPP_INFO(this->get_logger(), "Publishing imu data [%f %f %f] [%f %f %f]",
                message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z,
                message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z);
    publisher_imu_mpu_->publish(message);
}
