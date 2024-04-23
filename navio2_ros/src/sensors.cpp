#include "navio2_ros/sensors.h"

#include <chrono>
#include <functional>
#include <string>

using namespace std::chrono_literals;

NavioSensors::NavioSensors()
        : Node("navio_sensors"),
          imu_mpu_(Navio2::InertialSensorType::MPU9250),
          imu_lsm_(Navio2::InertialSensorType::LSM9DS1),
          baro_temp_(),
          gps_() {
    // TODO: enable IMUs with parameters

    // Publishers
    publisher_imu_mpu_ = this->create_publisher<ImuMsgAdapter>("imu/mpu", rclcpp::QoS(10).best_effort());
    publisher_imu_lsm_ = this->create_publisher<ImuMsgAdapter>("imu/lsm", rclcpp::QoS(10).best_effort());
    publisher_baro_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("barometer",
                                                                              rclcpp::QoS(10).best_effort());
    publisher_temp_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature",
                                                                            rclcpp::QoS(10).best_effort());
    publisher_gps_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/data", rclcpp::QoS(10).best_effort());
    publisher_gps_ublox_status_ = this->create_publisher<std_msgs::msg::Int8>("gps/ublox_status",
                                                                              rclcpp::QoS(10).best_effort());

    // Timers TODO: sensor publishing could be split with different timings
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

    // Barometer and temperature
    if (publisher_baro_->get_subscription_count() == 0 && publisher_temp_->get_subscription_count() == 0) {
        RCLCPP_DEBUG(this->get_logger(), "No subscribers, not processing barometer or temperature data");
    } else {
        float pressure, temperature;
        baro_temp_.read_data(pressure, temperature);
        publish_baro_data(pressure);
        publish_temp_data(temperature);
    }

    // GPS
    if (publisher_gps_->get_subscription_count() == 0 || publisher_gps_ublox_status_->get_subscription_count() == 0) {
        RCLCPP_DEBUG(this->get_logger(), "No subscribers, not processing GPS data");
    } else {
        bool some_update = gps_.read_data();
        if (some_update) {
            publish_gps_data();
        }
    }
}

void NavioSensors::process_imu(Navio2::InertialSensorType type) {
    auto imu_data = ImuData();
    imu_data.frame_id = "navio"; // TODO: get the frame id as param
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

void NavioSensors::publish_baro_data(const double &data) {
    RCLCPP_DEBUG(this->get_logger(), "Publishing barometer data: %f millibar", data);
    auto baro_msg = sensor_msgs::msg::FluidPressure();
    baro_msg.header.stamp = this->now();
    // Frame ID is useless, I suppose

    // fluid pressure unit is in Pascal, while data arrives in millibar
    baro_msg.fluid_pressure = data * 100.0;

    publisher_baro_->publish(baro_msg);
}

void NavioSensors::publish_temp_data(const double &data) {
    RCLCPP_DEBUG(this->get_logger(), "Publishing temperature data: %f Celsius", data);
    auto temp_msg = sensor_msgs::msg::Temperature();
    temp_msg.header.stamp = this->now();
    // Frame ID is useless, I suppose

    temp_msg.temperature = data;

    publisher_temp_->publish(temp_msg);

}

void NavioSensors::publish_gps_data() {
    RCLCPP_DEBUG(this->get_logger(), "Publishing GPS data");
    auto gps_msg = sensor_msgs::msg::NavSatFix();
    gps_msg.header.stamp = this->now();
    gps_msg.header.frame_id = "navio"; // TODO: get the frame id as param

    gps_msg.status.status = gps_.fixed ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                                       : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;

    gps_msg.latitude = gps_.data.latitude;
    gps_msg.longitude = gps_.data.longitude;
    gps_msg.altitude = gps_.data.height_above_ellipsoid;

    gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    gps_msg.position_covariance[0] = gps_.data.accuracy_horizontal * gps_.data.accuracy_horizontal;
    gps_msg.position_covariance[4] = gps_.data.accuracy_horizontal * gps_.data.accuracy_horizontal;
    gps_msg.position_covariance[8] = gps_.data.accuracy_vertical * gps_.data.accuracy_vertical;

    publisher_gps_->publish(gps_msg);

    std_msgs::msg::Int8 ublox_status;
    ublox_status.data = gps_.fix_status;
    publisher_gps_ublox_status_->publish(ublox_status);
}
