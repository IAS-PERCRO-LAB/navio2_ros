#include "navio2_ros/imu.h"

#include <memory>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

using namespace Navio2;

Imu::Imu(InertialSensorType type) {
    switch (type) {
        case InertialSensorType::MPU9250:
            sensor = std::unique_ptr<InertialSensor>{new MPU9250()};
            break;
        case InertialSensorType::LSM9DS1:
            sensor = std::unique_ptr<InertialSensor>{new LSM9DS1()};
            break;
        default:
            throw std::runtime_error("Unknown sensor type");
    }

    if (!sensor->probe()) {
        throw std::runtime_error("Sensor not enabled");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IMU enabled");
}

void Imu::read_data(std::array<double, 3> &accel, std::array<double, 3> &gyro) {
    sensor->update();
    sensor->read_accelerometer(reinterpret_cast<float *>(&accel[0]),
                               reinterpret_cast<float *>(&accel[1]),
                               reinterpret_cast<float *>(&accel[2]));
    sensor->read_gyroscope(reinterpret_cast<float *>(&gyro[0]),
                           reinterpret_cast<float *>(&gyro[1]),
                           reinterpret_cast<float *>(&gyro[2]));
}

void Imu::read_data(std::array<double, 3> &accel, std::array<double, 3> &gyro, std::array<double, 3> &mag) {
    Imu::read_data(accel, gyro);
    sensor->read_magnetometer(reinterpret_cast<float *>(&mag[0]),
                              reinterpret_cast<float *>(&mag[1]),
                              reinterpret_cast<float *>(&mag[2]));
}
