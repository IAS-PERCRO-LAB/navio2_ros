#ifndef NAVIO2_ROS_GPS_H
#define NAVIO2_ROS_GPS_H

/*
 * Receive position information with GPS driver as Ublox data.
 * Ublox GPS receiver is connected as an SPI device 0.0(/dev/spidev0.0).
 * The receiver sends information over SPI in an endless stream.
 *
 * See for additional implementation hints: https://github.com/IAS-PERCRO-LAB/Navio2/blob/main/C%2B%2B/Examples/gps.cpp
 */

#include <Common/Ublox.h>

namespace Navio2 {
    struct GpsData {
        double longitude;
        double latitude;
        double height_above_ellipsoid;  // meters
        double height_above_sea;  // meters
        double accuracy_horizontal;  // meters
        double accuracy_vertical;  // meters
    };

    /// GPS fix status. Numbers are mapped from Ublox NAV_STATUS data, don't change them unless you know what you're doing
    enum GpsFixStatus {
        NO_FIX = 0,
        DEAD_RECKONING_ONLY = 1,
        FIX_2D = 2,
        FIX_3D = 3,
        GPS_DEAD_RECKONING = 4,
        TIME_ONLY = 5,
        UNKNOWN = -1
    };

    class Gps {
    public:
        bool fixed;
        GpsFixStatus fix_status;
        GpsData data;

        Gps();

        /**
         * Update internal data.
         * @return true if a NAV_POSLLH or NAV_STATUS has been decoded and data or fixed info has been updated.
         */
        bool read_data();

    private:
        Ublox sensor_;
    };
}

#endif //NAVIO2_ROS_GPS_H
