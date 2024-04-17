#include "navio2_ros/baro_temp.h"

namespace Navio2 {
    BaroTemp::BaroTemp() {
        sensor = std::make_unique<MS5611>();
        sensor->initialize();
    }

    void BaroTemp::read_data(float &pressure, float &temperature) {
        sensor->refreshPressure();
        usleep(10000); // Waiting for pressure data ready
        sensor->readPressure();

        sensor->refreshTemperature();
        usleep(10000); // Waiting for temperature data ready
        sensor->readTemperature();

        sensor->calculatePressureAndTemperature();

        pressure = sensor->getPressure();
        temperature = sensor->getTemperature();
    }
}
