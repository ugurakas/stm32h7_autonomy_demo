#include "components/autonomy_controller.hpp"

#include <cstdint>

namespace drone::components {

AutonomyController::AutonomyController()
    : targetAltitude_(0.0f), targetHeading_(0.0f), timer_(0.0f),
      obstacleDistance_(0.0f), obstacleDetected_(false), missionComplete_(true) {
}

void AutonomyController::update(float dt) {
    timer_ += dt;
    
    // Basic attitude stabilization with gentle drift
    state_.throttle = 0.55f + 0.05f * (state_.pitch + state_.roll + state_.yaw);
    state_.pitch *= 0.98f;
    state_.roll *= 0.98f;
    state_.yaw *= 0.99f;
    
    // Update altitude hold
    updateAltitudeHold(dt);
    
    // Update heading hold
    updateHeadingHold(dt);
    
    // Check obstacle avoidance
    updateObstacleAvoidance(dt);
    
    // Simple mission pattern: change altitude periodically
    if (timer_ > 10.0f) {
        timer_ = 0.0f;
        targetAltitude_ = (targetAltitude_ > 1.0f) ? 0.5f : 2.0f;
    }
}

void AutonomyController::setMissionWaypoint(float alt, float heading) {
    targetAltitude_ = alt;
    targetHeading_ = heading;
    missionComplete_ = false;
}

bool AutonomyController::isMissionComplete() const noexcept {
    return missionComplete_;
}

bool AutonomyController::hasObstacle() const noexcept {
    return obstacleDetected_;
}

void AutonomyController::setObstacleDetected(bool detected, float distance) {
    obstacleDetected_ = detected;
    obstacleDistance_ = distance;
}

float AutonomyController::getTargetAltitude() const noexcept {
    return targetAltitude_;
}

void AutonomyController::setTargetAltitude(float meters) {
    targetAltitude_ = meters;
}

const core::FlightState& AutonomyController::state() const noexcept {
    return state_;
}

void AutonomyController::reset() noexcept {
    state_ = {};
    targetState_ = {};
    targetAltitude_ = 0.0f;
    targetHeading_ = 0.0f;
    timer_ = 0.0f;
    obstacleDistance_ = 0.0f;
    obstacleDetected_ = false;
    missionComplete_ = true;
}

void AutonomyController::updateAltitudeHold(float dt) {
    float altitudeError = targetAltitude_ - (state_.throttle * 2.0f - 1.0f) * 2.0f;
    float correction = altitudeError * 0.1f;
    state_.throttle += correction * dt;
    
    // Clamp throttle
    if (state_.throttle > 0.9f) state_.throttle = 0.9f;
    if (state_.throttle < 0.1f) state_.throttle = 0.1f;
}

void AutonomyController::updateHeadingHold(float dt) {
    float headingError = targetHeading_ - state_.yaw;
    while (headingError > 3.14159f) headingError -= 6.28318f;
    while (headingError < -3.14159f) headingError += 6.28318f;
    
    state_.yaw += headingError * 0.5f * dt;
}

void AutonomyController::updateObstacleAvoidance(float dt) {
    if (obstacleDetected_ && obstacleDistance_ < 1.0f) {
        // Simple avoidance: yaw away from obstacle
        state_.yaw += 0.5f * dt;
        // Reduce throttle to slow down
        if (state_.throttle > 0.4f) {
            state_.throttle = 0.4f;
        }
    }
}

} // namespace drone::components
