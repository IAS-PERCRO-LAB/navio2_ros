#ifndef NAVIO2_ROS_BARO_TEMP_H
#define NAVIO2_ROS_BARO_TEMP_H

// Get pressure from MS5611 barometer onboard of Navio shield.

#include <Common/MS5611.h>

#include <memory>

namespace Navio2 {
    class BaroTemp {
    public:
        BaroTemp();

        /// Read pressure (in millibar) and temperature (in Celsius) from the sensor.
        void read_data(float &pressure, float &temperature);
    private:
        std::unique_ptr<MS5611> sensor;
    };
}

#endif //NAVIO2_ROS_BARO_TEMP_H
