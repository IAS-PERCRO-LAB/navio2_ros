#include "navio2_ros/sensors/imu.h"

#include <memory>
#include <stdexcept>
#include <iostream>

using namespace Navio2;

Imu::Imu(InertialSensorType type) {
    std::cout << "Creating Imu object... ";

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

    sensor->initialize();

    std::cout << "done!" << std::endl;
}

void Imu::read_data(std::array<double, 3> &accel, std::array<double, 3> &gyro) {
    std::array<float, 3> accel_float{}, gyro_float{};

    sensor->update();
    sensor->read_accelerometer(&accel_float[0], &accel_float[1], &accel_float[2]);
    sensor->read_gyroscope(&gyro_float[0], &gyro_float[1], &gyro_float[2]);

    accel = {accel_float[0], accel_float[1], accel_float[2]};
    gyro = {gyro_float[0], gyro_float[1], gyro_float[2]};
}

void Imu::read_data(std::array<double, 3> &accel, std::array<double, 3> &gyro, std::array<double, 3> &mag) {
    std::array<float, 3> mag_float{};

    Imu::read_data(accel, gyro);

    sensor->read_magnetometer(&mag_float[0], &mag_float[1], &mag_float[2]);
    mag = {mag_float[0], mag_float[1], mag_float[2]};
}
