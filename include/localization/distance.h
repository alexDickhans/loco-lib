#pragma once

#include "sensorModel.h"
#include "utils.h"

const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> WALLS = {
	{{1.78308, 1.78308}, {1.78308, -1.78308}},
	{{1.78308, -1.78308}, {-1.78308, -1.78308}},
	{{-1.78308, -1.78308}, {-1.78308, 1.78308}},
	{{-1.78308, 1.78308}, {1.78308, 1.78308}},
};

constexpr float WALL_0_X = 1.78308;
constexpr float WALL_1_Y = 1.78308;
constexpr float WALL_2_X = -1.78308;
constexpr float WALL_3_Y = -1.78308;

class Distance : public SensorModel {
private:
	Eigen::Vector3f sensorOffset;
	pros::Distance distance;

	QLength measured = 0.0;
	bool exit = false;
	QLength std = 0.0;
public:
	Distance(Eigen::Vector3f sensor_offset, pros::Distance distance)
		: sensorOffset(std::move(sensor_offset)),
		  distance(std::move(distance)) {
	}

	void update() override {
		const auto measuredMM = distance.get();

		exit = measuredMM == 9999 || distance.get_object_size() < 70;

		measured = measuredMM * millimetre;

		std = 0.20 * measured / (distance.get_confidence() / 64.0);

	}

	[[nodiscard]] std::optional<double> p(const Eigen::Vector3f& X) override {

		if (exit) {
			return std::nullopt;
		}

		auto angle = X.z() + sensorOffset.z();

		Eigen::Vector2f x = X.head<2>() + Eigen::Rotation2Df(X.z()) * sensorOffset.head<2>();

		auto predicted = 50.0f;

		if (const auto theta = abs(std::remainder(0.0f, angle)); theta < M_PI_2) {
			predicted = std::min((WALL_0_X - x.x()) / cos(theta), predicted);
		}

		if (const auto theta = abs(std::remainder(static_cast<float>(M_PI_2), angle)); theta < M_PI_2) {
			predicted = std::min((WALL_1_Y - x.y()) / cos(theta), predicted);
		}

		if (const auto theta = abs(std::remainder(static_cast<float>(M_PI), angle)); theta < M_PI_2) {
			predicted = std::min((x.x() - WALL_2_X) / cos(theta), predicted);
		}

		if (const auto theta = abs(std::remainder(static_cast<float>(M_3PI_4), angle)); theta < M_PI_2) {
			predicted = std::min((x.y() - WALL_3_Y) / cos(theta), predicted);
		}

		return cheap_norm_pdf((predicted - measured.getValue())/std.getValue()) * LOCO_CONFIG::DISTANCE_WEIGHT;
	}

	~Distance() override = default;
};