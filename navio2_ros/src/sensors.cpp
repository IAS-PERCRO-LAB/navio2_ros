#include "navio2_ros/sensors.h"

#include <chrono>
#include <functional>
#include <string>

using namespace std::chrono_literals;

NavioSensors::NavioSensors()
        : Node("navio_sensors"),
          imu_mpu_(Navio2::InertialSensorType::MPU9250),
          imu_lsm_(Navio2::InertialSensorType::LSM9DS1) {
    // TODO: make the other one too and enable them with a parameter

    // Publishers
    publisher_imu_mpu_ = this->create_publisher<ImuMsgAdapter>("imu/mpu", rclcpp::QoS(10).best_effort());
    publisher_imu_lsm_ = this->create_publisher<ImuMsgAdapter>("imu/lsm", rclcpp::QoS(10).best_effort());

    // Timers
    timer_ = this->create_wall_timer(
            500ms, [this] { timer_callback(); });

    RCLCPP_DEBUG(this->get_logger(), "navio_sensors node started");
}

void NavioSensors::timer_callback() {
    // IMU MPU9250
    if (publisher_imu_mpu_->get_subscription_count() == 0) {
        RCLCPP_DEBUG(this->get_logger(), "No subscribers, not processing IMU MPU9250 data");
    } else {
        process_imu(Navio2::InertialSensorType::MPU9250);
    }

    // IMU LSM9DS1
    if (publisher_imu_lsm_->get_subscription_count() == 0) {
        RCLCPP_DEBUG(this->get_logger(), "No subscribers, not processing IMU LSM9DS1 data");
    } else {
        process_imu(Navio2::InertialSensorType::LSM9DS1);
    }
}

void NavioSensors::process_imu(Navio2::InertialSensorType type) {
    auto imu_data = ImuData();
    imu_data.frame_id = "navio"; // TODO: set the frame id according to the sensor
    imu_data.timestamp = this->now();

    switch (type) {
        case Navio2::InertialSensorType::MPU9250:
            imu_mpu_.read_data(imu_data.linear_acceleration, imu_data.angular_velocity);
            break;
        case Navio2::InertialSensorType::LSM9DS1:
            imu_lsm_.read_data(imu_data.linear_acceleration, imu_data.angular_velocity);
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown IMU type: %d", static_cast<int>(type));
            return;
    }

    // TODO: magnetic fields with https://docs.ros2.org/latest/api/sensor_msgs/msg/MagneticField.html

    // TODO: covariances!

    publish_imu_data(type, imu_data);
}

void NavioSensors::publish_imu_data(Navio2::InertialSensorType type, const ImuData &data) {
    RCLCPP_DEBUG(this->get_logger(),
                 "Publishing IMU %d data: linear_acceleration: [%f, %f, %f], angular_velocity: [%f, %f, %f]",
                 static_cast<int>(type),
                 data.linear_acceleration[0], data.linear_acceleration[1], data.linear_acceleration[2],
                 data.angular_velocity[0], data.angular_velocity[1], data.angular_velocity[2]);
    switch (type) {
        case Navio2::InertialSensorType::MPU9250:
            publisher_imu_mpu_->publish(data);
            break;
        case Navio2::InertialSensorType::LSM9DS1:
            publisher_imu_lsm_->publish(data);
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown IMU type: %d", static_cast<int>(type));
    }
    publisher_imu_mpu_->publish(data);
}
