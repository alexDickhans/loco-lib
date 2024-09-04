#pragma once

#include "config.h"
#include "sensor.h"
#include "config.h"

const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> LINES = {};

class LineSensor : public Sensor {
private:
	Eigen::Vector2d sensorOffset;
	pros::adi::LineSensor lineSensor;
public:
	LineSensor(Eigen::Vector2d sensor_offset, pros::adi::LineSensor line_sensor)
		: sensorOffset(std::move(sensor_offset)),
		  lineSensor(std::move(line_sensor)) {
	}

	std::optional<double> p(Eigen::Vector3d x) override {

		auto measured = this->lineSensor.get_value() > LOCO_CONFIG::LINE_SENSOR_THRESHOLD;
		auto sensor_position = Eigen::Rotation2Dd(x.z()) * sensorOffset + x.head<2>();

		auto predicted = 50.0_m;

		for (auto [fst, snd] : LINES) {
			predicted = std::min(((fst.y() - snd.y()) * sensor_position.y()
				- (fst.x() - snd.x()) * sensor_position.x()
				+ snd.x() * fst.y()
				- snd.y() * fst.x())
				/ (fst - snd).norm() * metre, predicted);
		}

		auto predictedBool = predicted < LOCO_CONFIG::LINE_SENSOR_DISTANCE_THRESHOLD;

		if (predictedBool && measured) {
			return 1.0 * LOCO_CONFIG::LINE_WEIGHT;
		} else if (!predictedBool && !measured) {
			return 1.0 * LOCO_CONFIG::LINE_WEIGHT;
		} else {
			return 0.0 * LOCO_CONFIG::LINE_WEIGHT;
		}
	}
};