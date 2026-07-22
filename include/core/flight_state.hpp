#pragma once

namespace drone::core {

struct FlightState {
    float throttle = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
};

} // namespace drone::core

namespace drone {
using core::FlightState;
} // namespace drone
