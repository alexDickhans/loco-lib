#pragma once

#include "config.h"
#include "sensor.h"
#include "utils/utils.h"

class GpsSensor : public Sensor {
private:
	pros::Gps gps;
	Angle sensorAngleOffset;
public:
	GpsSensor(pros::Gps gps, const Angle sensorAngleOffset)
		: gps(std::move(gps)),
		  sensorAngleOffset(sensorAngleOffset) {
	}

	std::optional<double> p(Eigen::Vector3d X) override {
		if (!gps.is_installed()) {
			return std::nullopt;
		}

		// TODO: Flag stuff

		auto [x, y] = gps.get_position();

		const auto std = gps.get_error();

		const auto point = Eigen::Vector2d(x, y);
		const auto predicted = Eigen::Vector2d(X.x(), X.y());

		return normal_pdf((point - predicted).norm(), 0.0, std) * LOCO_CONFIG::GPS_WEIGHT;
	}

	~GpsSensor() override = default;
};