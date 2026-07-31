#include "drivers/imu_sensor.hpp"

namespace drone::drivers {

// Initialize the simulated IMU sensor.
void MockImuSensor::init() {
}

ImuReading MockImuSensor::read() {
    // Instance member, not a function-local static: a function-local
    // static would be shared across every MockImuSensor instance.
    angle_ += 0.01f;
    return ImuReading{
        0.0f,
        0.0f,
        1.0f,
        0.0f,
        0.0f,
        angle_,
        25.0f
    };
}

} // namespace drone::drivers
