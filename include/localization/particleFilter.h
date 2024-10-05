#pragma once

#include "Eigen/Eigen"
#include "units/units.hpp"
#include "sensorModel.h"

#include <random>
#include <algorithm>

#include "config.h"

namespace loco {
    /**
     * @brief Initializes a particle filter with a pre-specified number of particles.
     *
     * @warning Due to current efficiency limitations, the particle limit is 500 (Takes approximately 6ms to compute each
     * frame). For calculating frame time, estimate processing time to be 12µs/particle.
     *
     * @tparam L Number of particle to initialize the filter with. More is generally better for accuracy, however there are
     * diminishing returns once the particle count is greater than 100, view warning for notes on large particle quantities.
     */
    template<size_t L>
    class ParticleFilter {
        // Ensure particles are less than the max of 500 particles
        static_assert(std::less_equal<size_t>()(L, 500));

    private:
        /**
         *
         */
        std::array<std::array<float, 2>, L> particles;
        std::array<std::array<float, 2>, L> oldParticles;
        std::array<float, L> weights;

        Eigen::Vector3f prediction{};

        std::vector<SensorModel *> sensors;

        QLength distanceSinceUpdate = 0.0;
        QTime lastUpdateTime = 0.0;

        QLength maxDistanceSinceUpdate = 1_in;
        QTime maxUpdateInterval = 2_s;

        std::function<Angle()> angleFunction;
        std::ranlux24_base de;

        std::uniform_real_distribution<> fieldDist{-1.78308, 1.78308};

    public:
        explicit ParticleFilter(std::function<Angle()> angle_function)
            : angleFunction(std::move(angle_function)) {
            for (auto &&particle: particles) {
                particle[0] = 0.0;
                particle[1] = 0.0;
            }
        }

        Eigen::Vector3f getPrediction() {
            return prediction;
        }

        std::array<Eigen::Vector3f, L> getParticles() {
            std::array<Eigen::Vector3f, L> particles;

            const Angle angle = angleFunction();

            for (size_t i = 0; i < L; i++) {
                particles[i] = Eigen::Vector3f(this->particles[i][0], this->particles[i][1], angle.getValue());
            }

            return particles;
        }

        Eigen::Vector3f getParticle(size_t i) {
            return {this->particles[i][0], this->particles[i][1], angleFunction().getValue()};
        }

        void update(const std::function<Eigen::Vector2f()> &predictionFunction) {
            if (!isfinite(angleFunction().getValue())) {
                return;
            }

            auto start = pros::micros();

            const Angle angle = angleFunction();

            for (auto &&particle: particles) {
                auto prediction = predictionFunction();
                particle[0] += prediction.x();
                particle[1] += prediction.y();
            }

            distanceSinceUpdate += predictionFunction().norm();

            if (distanceSinceUpdate < maxDistanceSinceUpdate && maxUpdateInterval > pros::millis() * millisecond) {
                return;
            }

            for (auto &&sensor: this->sensors) {
                sensor->update();
            }

            double totalWeight = 0.0;

            for (size_t i = 0; i < L; i++) {
                weights[i] = 1.0;

                if (outOfField(particles[i])) {
                    particles[i][0] = fieldDist(de);
                    particles[i][1] = fieldDist(de);
                }

                auto particle = Eigen::Vector3f(particles[i][0], particles[i][1], angle.getValue());

                for (const auto sensor: sensors) {
                    if (auto weight = sensor->p(particle); weight.has_value() && isfinite(weight.value())) {
                        weights[i] = weights[i] * weight.value();
                    }
                }

                weights[i] = weights[i];

                totalWeight = totalWeight + weights[i];
            }

            if (totalWeight == 0.0) {
                std::cout << "Warning: Total weight equal to 0" << std::endl;
                return;
            }

            const double avgWeight = totalWeight / static_cast<double>(L);

            std::uniform_real_distribution distribution(0.0, avgWeight);
            const double randWeight = distribution(de);

            for (size_t i = 0; i < particles.size(); i++) {
                oldParticles[i] = particles[i];
            }

            size_t j = 0;
            auto cumulativeWeight = 0.0;

            float xSum = 0.0, ySum = 0.0;

            for (size_t i = 0; i < L; i++) {
                const auto weight = static_cast<double>(i) * avgWeight + randWeight;

                while (cumulativeWeight < weight) {
                    if (j >= weights.size()) {
                        break;
                    }
                    cumulativeWeight += weights[j];
                    j++;
                }

                particles[i][0] = oldParticles[j - 1][0];
                particles[i][1] = oldParticles[j - 1][1];

                xSum += particles[i][0];
                ySum += particles[i][1];
            }

            prediction = Eigen::Vector3f(xSum / static_cast<float>(L), ySum / static_cast<float>(L), angle.getValue());

            lastUpdateTime = pros::millis() * millisecond;
            distanceSinceUpdate = 0.0;
        }

        void initNormal(const Eigen::Vector2f &mean, const Eigen::Matrix2f &covariance, const bool flip) {
            for (auto &&particle: this->particles) {
                Eigen::Vector2f p = mean + covariance * Eigen::Vector2f::Random();
                particle[0] = p.x();
                particle[1] = p.y() * (flip ? -1.0 : 1.0);
            }

            prediction.z() = angleFunction().getValue();
            distanceSinceUpdate += 2.0 * distanceSinceUpdate;
        }

        static bool outOfField(const std::array<float, 2> &vector) {
            return vector[0] > 1.78308 || vector[0] < -1.78308 || vector[1] < -1.78308 || vector[1] > 1.78308;
        }

        void initUniform(const QLength minX, const QLength minY, const QLength maxX, const QLength maxY) {
            std::uniform_real_distribution xDistribution(minX.getValue(), maxX.getValue());
            std::uniform_real_distribution yDistribution(minY.getValue(), maxY.getValue());

            for (auto &&particle: this->particles) {
                particle[0] = xDistribution(de);
                particle[1] = yDistribution(de);
            }
        }

        void addSensor(SensorModel *sensor) {
            this->sensors.emplace_back(sensor);
        }

        Angle getAngle() {
            return angleFunction();
        }
    };
}
