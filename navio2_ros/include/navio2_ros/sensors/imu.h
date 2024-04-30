#ifndef NAVIO2_ROS_IMU_H
#define NAVIO2_ROS_IMU_H

/*
 * Read accelerometer, gyroscope and magnetometer values from inertial measurement unit: MPU9250 or LSM9DS1 over SPI.
 * Navio's onboard sensors are connected to the SPI bus on Raspberry Pi and can be read through:
 * - /dev/spidev0.1 (MPU9250)
 * - /dev/spidev0.2 (mag LSM9DS1)
 * - /dev/spidev0.3 (acc/gyro LSM9DS1)
 */

#include <Common/MPU9250.h>
#include <Navio2/LSM9DS1.h>

#include <memory>

namespace Navio2 {
    enum class InertialSensorType {
        MPU9250,
        LSM9DS1
    };

    class Imu {
    public:
        explicit Imu(InertialSensorType type);

        void read_data(std::array<double, 3> &accel, std::array<double, 3> &gyro);

        void read_data(std::array<double, 3> &accel, std::array<double, 3> &gyro, std::array<double, 3> &mag);

    private:
        std::unique_ptr<InertialSensor> sensor;
    };
}

#endif //NAVIO2_ROS_IMU_H
