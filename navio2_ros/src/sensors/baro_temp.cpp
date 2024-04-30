#include "navio2_ros/sensors/baro_temp.h"

namespace Navio2 {
    BaroTemp::BaroTemp() {
        std::cout << "Creating BaroTemp object...";

        sensor = std::make_unique<MS5611>();
        sensor->initialize();

        std::cout << "done!" << std::endl;
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
