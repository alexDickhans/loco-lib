#pragma once

namespace loco {
    /**
     * @brief Defiens a SensorModel to be used in the \refitem ParticleFilter. This is used in the update step to revise the filter's belief.
     */
    class SensorModel {
    public:
        /**
         * @brief Determine p(z_k, x_k). This is the chance that this sensor reading (z_k) would occur given the robot would be at that particle (x_k).
         *
         * @param x The current particle's position
         * @return Result of p(z_k, x_k)
         */
        virtual std::optional<double> p(const Eigen::Vector3f &x) = 0;

        /**
         * @brief Update object with sensor readings, this function is called every frame (~10ms) and should be used to stash expensive, one time computations before all the particles are calculated, this always runs before p(x) each frame.
         */
        virtual void update() = 0;

        virtual ~SensorModel() = default;
    };
}
