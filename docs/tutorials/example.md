# Annotated Example

In this example we will make the code necessary for a Particle Filter. This is after you have installed Loco lib into
your local project. If it's not in your local project, use the [installation tutorial](./installation.md) to install it.

## Example Prerequisites

For this example you need to understand Basic C++ Programming, but more specifically, anonymous functions, object usage,
and lambda expressions. If you need to learn
the [learncpp.com](https://www.learncpp.com/cpp-tutorial/introduction-to-object-oriented-programming/) articles are
great places to learn!

## Setup

### Configuration

While you can set the configuration values when you use them, it is often helpful to have them all in the same spot.
For example, putting them at the top of the main.cpp, or another accessible spot can help with ease of use.

These tuning variables pertain just to how the drivetrain calculates how far it has travelled.

```c++
constexpr float DRIVE_RATIO = 48.0/36.0; // EX: 36 tooth driving gear to 48 tooth driven gear.
constexpr double DRIVETRAIN_TUNING_SCALAR = 76.0/87.9; // Tuning variable to make sure distance matches
constexpr QLength WHEEL_RADIUS = 3.25_in/2.0; // Wheel radius
```

These values are the noise values, we will discuss the noising process later in this tutorial, but it is crucial to a
working particle filter and must be > 0. We recommend you use similar values to these:

```c++
constexpr float DRIVE_NOISE = 0.35; // The desired amount in % of noise on the drive
constexpr Angle ANGLE_NOISE = 8_deg; // The noise on the angle that's desired
```

### Hardware definitions

Similar to configuration values, it's recommended that you put the port definitions at a easy to remember spot. In this
example we put them directly below the configuration values. A special note here is that there is a distance sensor
offset. This allows the distance sensor to be put anywhere on the robot and still give consistent positions. To ensure
we have reliable sensor positions we use a CAD drawing such as the following from our notebook to define the sensor
positions:

![A drawing of the sensor offset on our robot](../images/sensorDrawing.png)

```c++
pros::MotorGroup left11W({-2, -3}), right11W({6, 7}); // left and right motor setup

// Create IMU and distance sensors
pros::Imu imu(19);
pros::Distance distance(18);

// Distance sensor pointed to the left, 5.8 inches to the left, and 4.2 inches back from the center of rotation
// A different distance offset needs to be used for each distance sensor
const Eigen::Vector3f DISTANCE_OFFSET((-4.2_in).getValue(), (5.8_in).getValue(), (90_deg).getValue());
```

### Particle filter initialization

Currently the particle filter implementation is in a form that is very flexible to the addition or utilization of
different sensors. This means that certain parts of it use lambda expressions to make the code simpler. The following
code creates a particle filter with 100 particles that uses the IMU laid flat as it's orientation source.

```c++
// Create a particle filter with 100 particles, and an angle function, given above
loco::ParticleFilter<100> particleFilter([]() {
    // Invert the angle into the loco coordinate system
    const Angle angle = -imu.get_rotation() * degree;

    // Check to make sure the angle isn't nan, if it is it can cause issues in the position change calculations
    return isfinite(angle.getValue()) ? angle : 0.0;
});
```

### Misc. Setup

Particle filters need a lot of noise to work properly, which we will describe in more detail later in this example, so
we use a ranlux24_base random number generator as our default engine (de).

```c++
// Cheap (computationally) random generator to add randomness to the drivetrain
std::ranlux24_base de;
```

Another thing we have to setup is the variables to store the last drivetrain state so it's possible to calculate the
change each frame. We will use this later in the tutorial.

```c++
// Used to calculate the change in position on the drivetrain
QLength lastLeft, lastRight;
```

Another thing we set up is a function to calculate the distance travelled of the drivetrain. To do this we use a function that takes in motor groups and calculates the average position, which we can use later to calculate the change in position in the local, and then global frames. 

```c++
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
```

## Initialize

First, calibrate the IMU:

```c++
// Calibrate the IMU and wait until it is done
imu.reset(true);
```

Add the distance sensor to the particle filter:

```c++
// Add the distance sensor to the particle filter
particleFilter.addSensor(new loco::DistanceSensorModel(DISTANCE_OFFSET, distance));
```

Initialize the particles in a uniform distribution around the field:

```c++
// Initialize the starting distribution over the entire field in a uniform manner.
particleFilter.initUniform(-70_in, -70_in, 70_in, 70_in);
```

### Making a localization task

Localization can be run many points in code, but we recommend that you give it it's own task, or add it to subsystem code in [command based pros](https://github.com/alexDickhans/command-based-pros). For this example we put create an independently scheduled task for this:

```c++
// Create a task to run localization
pros::Task locoTask = pros::Task([&]() {
```

First, we have the loop code. This makes sure the code is run on a consistent schedule:

```c++
uint32_t start_time = 0;

// Run localization forever
while (true) {
    // Store the start time to ensure that the time between updates remains consistent
    start_time = pros::millis();
```

We then get the distance travelled by the drivetrain, and subtract it from the distance last frame to estimate the change:

```c++
// Store the current distance of the drivetrain
const QLength leftLength = getDistance(left11W);
const QLength rightLength = getDistance(right11W);

// Calculate the change from the previous position
const QLength leftChange = leftLength - lastLeft;
const QLength rightChange = rightLength - lastRight;
```

Then, store the current drivetrain position into the last state to be used in the future:

```c++
// Store the current value as the last value for next frame
lastLeft = leftLength;
lastRight = rightLength;
```

The average distance change in the drivetrain is calculated, on tank drives, this gets the movement in the middle of the robot. 

```c++
// Calculate the average movement, this is a cheaper way to get the movement of the drive at the center for
// skid-steer based mechanics
auto avg = (leftChange + rightChange) / 2.0;
```

The next step is to define the random distribution objects we will sample from in the prediction step of the filter.

```c++
// Define the distributions to add noise to the sensor readings
std::uniform_real_distribution avgDistribution(avg.getValue() - DRIVE_NOISE * avg.getValue(),
                                               avg.getValue() + DRIVE_NOISE * avg.getValue());
std::uniform_real_distribution angleDistribution(
    particleFilter.getAngle().getValue() - ANGLE_NOISE.getValue(),
    particleFilter.getAngle().getValue() + ANGLE_NOISE.getValue());
```

We then update the filter with this function, which samples the distributions we just defined creates a local translation and rotates it into the global frame. 

```c++
// Update the filter with the new data and noise from the sensors
particleFilter.update([&]() mutable {
    // Calculate noisy sensor readings
    const auto noisy = avgDistribution(de);
    const auto angle = angleDistribution(de);

    // Calculate the translation with the sensor readings
    return Eigen::Rotation2Df(angle) * Eigen::Vector2f({noisy, 0.0});
});
```

Last, we finish off the loop by waiting 10ms from when the loop started:

```c++
        // Wait 10ms for the next frame, incorporating the wait
        pros::c::task_delay_until(&start_time, 10);
    }
});
```

### *Notes on the prediction function*

! TODO

## Getting the predictions

In the opcontrol() function, we use the particleFilter.getPrediction() which returns the average particle from the filter. This returns a Eigen::Vector3f, which you can easily get the [x, y, ø] from in SI units.

```c++
// Get the current pose prediction from the particle filter.
auto pose = particleFilter.getPrediction();

// Print out the current pose of the robot to the screen in inches and degrees. The outputs from the particle
// filter are always in base SI units, so meters for position, seconds for time, and radians for angle.
pros::lcd::set_text(2, std::to_string(pose.x() * metre.Convert(inch)) + ", " +
```

And that's all you need for a tank drive implementation of the particle filter!
