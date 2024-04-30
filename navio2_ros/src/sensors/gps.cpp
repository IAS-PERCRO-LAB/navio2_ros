#include "navio2_ros/sensors/gps.h"

namespace Navio2 {
    Gps::Gps() : fixed(false), fix_status(UNKNOWN) {
        std::cout << "Creating Gps object...";

        // testConnection() waits for a ubx protocol message and checks if there's at least one correct message in the first 300 symbols
        if (!sensor_.testConnection()) {
            throw std::runtime_error("Ublox test not passed\nAbort program!\n");
        }

        if (!sensor_.configureSolutionRate(1000)) {
            throw std::runtime_error("Setting new rate: FAILED\n");
        }

        std::cout << "done!" << std::endl;
    }

    bool Gps::read_data() {
        std::vector<double> pos_data;
        bool updated = false;

        // NAV_POSLLH message provides Latitude, Longitude, Height above Ellipsoid and Height above MSL
        if (sensor_.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data) == 1) {
            // the way data is stored in pos_data vector is specified in decodeMessage() of class UBXParser (look for ublox.h)

            // pos_data[0] is GPS Millisecond Time of Week
            data.longitude = pos_data[1] / 10000000;
            data.latitude = pos_data[2] / 10000000;
            data.height_above_ellipsoid = pos_data[3] / 1000;
            data.height_above_sea = pos_data[4] / 1000;
            data.accuracy_horizontal = pos_data[5] / 1000;
            data.accuracy_vertical = pos_data[6] / 1000;

            updated = true;
        }

        // NAV_STATUS message provides information about the reception of the GPS signal
        if (sensor_.decodeSingleMessage(Ublox::NAV_STATUS, pos_data) == 1) {
            fixed = (bool) ((int) pos_data[1] & 0x01);

            // safe-cast the fix status
            if ((int) pos_data[0] >= 0 && (int) pos_data[0] <= 5) {
                fix_status = (GpsFixStatus) ((int) pos_data[0]);
            } else {
                fix_status = UNKNOWN;
            }

            updated = true;
        }

        return updated;
    }
}
