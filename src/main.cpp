#include "main.h"

constexpr float DRIVE_RATIO = 48.0/36.0; // EX: 36 tooth driving gear to 48 tooth driven gear.
constexpr double DRIVETRAIN_TUNING_SCALAR = 76.0/87.9; // Tuning variable to make sure distance matches
constexpr QLength WHEEL_RADIUS = 3.25_in/2.0; // Wheel radius
constexpr float DRIVE_NOISE = 0.35; // The desired amount in % of noise on the drive
constexpr Angle ANGLE_NOISE = 8_deg; // The noise on the angle that's desired

pros::MotorGroup left11W({-2, -3}), right11W({6, 7}); // left and right motor setup

// Create IMU and distance sensors
pros::Imu imu(19);
pros::Distance distance(18);

// Distance sensor pointed to the left, 5.8 inches to the left, and 4.2 inches back from the center of rotation
// A different distance offset needs to be used for each distance sensor
const Eigen::Vector3f DISTANCE_OFFSET((-4.2_in).getValue(), (5.8_in).getValue(), (90_deg).getValue());

// Create a particle filter with 100 particles, and an angle function, given above
loco::ParticleFilter<100> particleFilter([]() {
    // Invert the angle into the loco coordinate system
    const Angle angle = -imu.get_rotation() * degree;

    // Check to make sure the angle isn't nan, if it is it can cause issues in the position change calculations
    return isfinite(angle.getValue()) ? angle : 0.0;
});

// Cheap random generator to add randomness to the drivetrain
std::ranlux24_base de;

// Used to calculate the change in position on the drivetrain
QLength lastLeft, lastRight;

/**
 * @brief get the average position of the motor group
 *
 * @param motor Motor group to get the distance travelled on
 * @return average position of the motor group
 */
QLength getDistance(const pros::MotorGroup& motor) {
    QLength totalPosition = 0.0;

    for (double position : motor.get_position_all()) {
        totalPosition += position / DRIVE_RATIO * 2.0 * M_PI
           * DRIVETRAIN_TUNING_SCALAR * WHEEL_RADIUS;
    }

    return totalPosition/motor.size();
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");

    pros::lcd::register_btn1_cb(on_center_button);

    // Calibrate the IMU and wait until it is done
    imu.reset(true);

    // Add the distance sensor to the particle filter
    particleFilter.addSensor(new loco::DistanceSensorModel(DISTANCE_OFFSET, distance));

    // Initialize the starting distribution over the entire field in a uniform manner.
    particleFilter.initUniform(-70_in, -70_in, 70_in, 70_in);

    // Create a task to run localization
    pros::Task locoTask = pros::Task([&]() {
        uint32_t start_time = 0;

        // Run localization forever
        while (true) {
            // Store the start time to ensure that the time between updates remains consistent
            start_time = pros::millis();

            // Store the current distance of the drivetrain
            const QLength leftLength = getDistance(left11W);
            const QLength rightLength = getDistance(right11W);

            // Calculate the change from the previous position
            const QLength leftChange = leftLength - lastLeft;
            const QLength rightChange = rightLength - lastRight;

            // Store the current value as the last value for next frame
            lastLeft = leftLength;
            lastRight = rightLength;

            // Calculate the average movement, this is a cheaper way to get the movement of the drive at the center for
            // skid-steer based mechanics
            auto avg = (leftChange + rightChange) / 2.0;

            // Define the distributions to add noise to the sensor readings
            std::uniform_real_distribution avgDistribution(avg.getValue() - DRIVE_NOISE * avg.getValue(),
                                                           avg.getValue() + DRIVE_NOISE * avg.getValue());
            std::uniform_real_distribution angleDistribution(
                particleFilter.getAngle().getValue() - ANGLE_NOISE.getValue(),
                particleFilter.getAngle().getValue() + ANGLE_NOISE.getValue());

            // Update the filter with the new data and noise from the sensors
            particleFilter.update([&]() mutable {
                // Calculate noisy sensor readings
                const auto noisy = avgDistribution(de);
                const auto angle = angleDistribution(de);

                // Calculate the translation with the sensor readings
                return Eigen::Rotation2Df(angle) * Eigen::Vector2f({noisy, 0.0});
            });

            // Wait 10ms for the next frame, incorporating the wait
            pros::c::task_delay_until(&start_time, 10);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (true) {
        // Get the current pose prediction from the particle filter.
        auto pose = particleFilter.getPrediction();

        // Print out the current pose of the robot to the screen in inches and degrees. The outputs from the particle
        // filter are always in base SI units, so meters for position, seconds for time, and radians for angle.
        pros::lcd::set_text(2, std::to_string(pose.x() * metre.Convert(inch)) + ", " +
                                       std::to_string(pose.y() * metre.Convert(inch)) + ", " +
                                       std::to_string(pose.z() * radian.Convert(degree)));

        // Arcade control scheme
        int dir = master.get_analog(ANALOG_LEFT_Y); // Gets amount forward/backward from left joystick
        int turn = master.get_analog(ANALOG_RIGHT_X); // Gets the turn left/right from right joystick
        left11W.move(dir - turn); // Sets left motor voltage
        right11W.move(dir + turn); // Sets right motor voltage

        pros::delay(20); // Run for 20 ms then update
    }
}
