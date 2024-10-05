#pragma once

namespace loco {
    /**
     * @brief Static class to hold configuration values for the particle filter. Generally, these values are a good
     * starting point and will work in most use cases, but that can also be changed as needed.
     */
    class LOCO_CONFIG {
    public:
        /**
         * @brief Sensor threshold for determining if the line sensor is over a line. This is a good value at approximately
         * 7mm from the tiles, which we found was a good balance between packaging and sensor SNR.
         */
        static constexpr size_t LINE_SENSOR_THRESHOLD = 2000;

        /**
         * @brief Distance from the CENTER of the line to predict the value to be positive. Largely untested, but upon more
         * testing this value will be updated appropriately.
         */
        static constexpr QLength LINE_SENSOR_DISTANCE_THRESHOLD = 1_in;

        /**
         * @brief Weight for the distance sensor. Higher means it will have a larger impact on the particle filter, lower
         * means a smaller impact.
         */
        static constexpr float DISTANCE_WEIGHT = 1.0;

        /**
         * @brief Weight for the game positioning sensor. Higher means it will have a larger impact on the particle filter, lower
         * means a smaller impact.
         */
        static constexpr float GPS_WEIGHT = 1.0;

        /**
         * @brief Weight for the line sensor. Higher means it will have a larger impact on the particle filter, lower
         * means a smaller impact.
         */
        static constexpr float LINE_WEIGHT = 1.0;
    };
}
