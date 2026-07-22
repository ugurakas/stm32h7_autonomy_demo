#include "drivers/imu_sensor.hpp"

namespace drone::drivers {

// Initialize the simulated IMU sensor.
void MockImuSensor::init() {
}

ImuReading MockImuSensor::read() {
    static float angle = 0.0f;
    angle += 0.01f;
    return ImuReading{
        0.0f,
        0.0f,
        1.0f,
        0.0f,
        0.0f,
        angle,
        25.0f
    };
}

} // namespace drone::drivers
