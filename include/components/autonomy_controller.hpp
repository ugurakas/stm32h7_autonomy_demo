#pragma once

#include "core/flight_state.hpp"
#include "core/drone_types.hpp"

namespace drone::components {

class AutonomyController {
public:
    AutonomyController();
    
    void update(float dt);
    const core::FlightState& state() const noexcept;
    void reset() noexcept;
    
    // Mission control
    void setMissionWaypoint(float alt, float heading);
    bool isMissionComplete() const noexcept;
    
    // Obstacle avoidance (placeholder)
    bool hasObstacle() const noexcept;
    void setObstacleDetected(bool detected, float distance);
    
    // Altitude hold
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

} // namespace drone::components

namespace drone {
using components::AutonomyController;
} // namespace drone
