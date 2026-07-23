/**
 * @file    autonomy_controller.hpp
 * @brief   Autonomous flight controller — altitude hold, heading hold,
 *          obstacle avoidance, and mission waypoint navigation.
 *
 * @details Provides a lightweight autonomous flight mode:
 *          - Altitude hold with PID-like correction
 *          - Heading hold with yaw error normalisation
 *          - Obstacle avoidance (placeholder) with throttle reduction
 *          - Simple periodic waypoint mission (altitude cycling)
 *          - Attitude stabilisation with gentle drift damping
 *
 * @ingroup components
 */

#pragma once

#include "core/flight_state.hpp"
#include "core/drone_types.hpp"

namespace drone::components {

/**
 * @defgroup autonomy_controller Autonomy Controller
 * @brief    Lightweight autonomous flight mode.
 *
 * ### Usage
 * @code{.cpp}
 *   AutonomyController ac;
 *   ac.setTargetAltitude(2.0f);
 *   ac.setMissionWaypoint(1.5f, 0.0f);
 *   ac.update(0.01f);
 *   const auto& state = ac.state();
 * @endcode
 *
 * @{
 */

class AutonomyController {
public:
    AutonomyController();

    /// @brief  Main update — runs altitude hold, heading hold, and obstacles.
    void update(float dt);

    /// @return Current flight state (throttle, roll, pitch, yaw).
    const core::FlightState& state() const noexcept;

    /// @brief  Reset all internal state.
    void reset() noexcept;

    // ---- Mission waypoint ----
    void setMissionWaypoint(float alt, float heading);
    bool isMissionComplete() const noexcept;

    // ---- Obstacle avoidance ----
    bool hasObstacle() const noexcept;
    void setObstacleDetected(bool detected, float distance);

    // ---- Altitude hold ----
    float getTargetAltitude() const noexcept;
    void setTargetAltitude(float meters);

private:
    core::FlightState state_{};
    core::FlightState targetState_{};
    float targetAltitude_;
    float targetHeading_;
    float timer_;
    float obstacleDistance_;
    bool obstacleDetected_;
    bool missionComplete_;

    void updateAltitudeHold(float dt);
    void updateHeadingHold(float dt);
    void updateObstacleAvoidance(float dt);
};

/** @} */  // end of autonomy_controller group

} // namespace drone::components

namespace drone {
using components::AutonomyController;
} // namespace drone
