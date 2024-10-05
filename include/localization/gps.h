#pragma once

#include "config.h"
#include "sensorModel.h"
#include "utils.h"

namespace loco {
    /**
     * @brief A sensor model for the VEX game positioning system. Uses the current position of the sensor and the
     * .getError() value to determine the confidence of the points
     */
    class GpsSensorModel : public SensorModel {
    private:
        pros::Gps gps;
        Angle sensorAngleOffset;
        Eigen::Vector2f point{};
        double std{0.0};
        bool notInstalled{false};

    public:
        /**
         *
         * @param sensorAngleOffset The offset ø of the robot, in the local coordinate system
         * @param gps A pros::Gps object, moved in the constructor
         */
        GpsSensorModel(const Angle sensorAngleOffset, pros::Gps gps)
            : gps(std::move(gps)),
              sensorAngleOffset(sensorAngleOffset) {
        }

        void update() override {
            notInstalled = !gps.is_installed() || gps.get_error() > 0.015;
            auto [x, y] = gps.get_position();

            point = Eigen::Vector2f(-y, x);

            std = gps.get_error() * 8.0;
        }

        std::optional<double> p(const Eigen::Vector3f &X) override {
            if (notInstalled) [[unlikely]] {
                return std::nullopt;
            }

            return cheap_norm_pdf(sqrt(X.x() * point.x() + X.y() * point.y()) / 2.0f) * LOCO_CONFIG::GPS_WEIGHT;
        }

        /**
         * Get the angle directly from the GPS sensor in the locolib coordinate system
         *
         * @return Angle in the Loco lib coordinate system
         */
        Angle getAngle() {
            return -gps.get_yaw() * 1_deg - sensorAngleOffset;
        }

        ~GpsSensorModel() override = default;
    };
}
