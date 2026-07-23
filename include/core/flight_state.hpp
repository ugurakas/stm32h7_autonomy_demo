/**
 * @file    flight_state.hpp
 * @brief   Core flight state — attitude and throttle setpoints.
 *
 * @details Defines the @ref FlightState struct used to pass attitude
 *          setpoints (roll, pitch, yaw, throttle) between the autonomy
 *          controller and the flight controller.
 *
 * @ingroup core
 */

#pragma once

namespace drone::core {

/**
 * @brief  Attitude setpoint / state with throttle.
 */
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
