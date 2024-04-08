#include "navio2_ros/imu.h"

#include <memory>
#include <stdexcept>

using namespace Navio2;

Imu::Imu(InertialSensorType type) {
    switch (type) {
        case InertialSensorType::MPU9250:
            sensor = new MPU9250();
            break;
        case InertialSensorType::LSM9DS1:
            sensor = new LSM9DS1();
            break;
        default:
            throw std::runtime_error("Unknown sensor type");
    }

    if (!sensor->probe()) {
        throw std::runtime_error("Sensor not enabled");
    }
}

void Imu::read_accelerometer(float *ax, float *ay, float *az) {
    sensor->read_accelerometer(ax, ay, az);
}

void Imu::read_gyroscope(float *gx, float *gy, float *gz) {
    sensor->read_gyroscope(gx, gy, gz);
}

void Imu::read_magnetometer(float *mx, float *my, float *mz) {
    sensor->read_magnetometer(mx, my, mz);
}
