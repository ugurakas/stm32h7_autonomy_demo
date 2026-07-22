#include "components/flight_controller.hpp"
#include "drivers/system_clock.hpp"

#include <cstdint>

namespace drone::components {

FlightController::FlightController()
    : internalState_(InternalState::Disarmed),
      failsafeActive_(false),
      failsafeTimer_(0),
      targetAltitude_(0.0f),
      altitudeIntegral_(0.0f) {
    
    // Configure PID output limits for actuator range
    rollPid_.setOutputLimits(-0.5f, 0.5f);
    pitchPid_.setOutputLimits(-0.5f, 0.5f);
    yawPid_.setOutputLimits(-0.3f, 0.3f);
    altitudePid_.setOutputLimits(-0.3f, 0.3f);
    
    // Configure integral limits
    rollPid_.setIntegralLimits(-0.2f, 0.2f);
    pitchPid_.setIntegralLimits(-0.2f, 0.2f);
    yawPid_.setIntegralLimits(-0.1f, 0.1f);
    altitudePid_.setIntegralLimits(-0.1f, 0.1f);
}

void FlightController::update(float dt) {
    // Check failsafe heartbeat timeout (100ms = 10Hz expected)
    if (failsafeActive_) {
        applyFailsafe();
        state_.mode = core::FlightMode::Fault;
        state_.heartbeat++;
        return;
    }
    
    if (internalState_ == InternalState::Disarmed) {
        // Outputs should be zero when disarmed
        state_.throttle = 0.0f;
        state_.mode = core::FlightMode::Idle;
        state_.heartbeat++;
        return;
    }
    
    if (pendingCommand_.type == core::CommandType::None && 
        internalState_ == InternalState::Armed && 
        state_.mode == core::FlightMode::Armed) {
        // Hold current attitude
        state_.heartbeat++;
        return;
    }
    
    switch (pendingCommand_.type) {
        case core::CommandType::Arm: {
            if (internalState_ == InternalState::Disarmed) {
                internalState_ = InternalState::Armed;
                state_.mode = core::FlightMode::Armed;
                // Reset PIDs on arm to prevent integral windup
                rollPid_.reset();
                pitchPid_.reset();
                yawPid_.reset();
                altitudePid_.reset();
                altitudeIntegral_ = 0.0f;
                targetAltitude_ = state_.altitudeMeters;
            }
            break;
        }
        case core::CommandType::Disarm: {
            internalState_ = InternalState::Disarmed;
            state_.mode = core::FlightMode::Idle;
            state_.throttle = 0.0f;
            state_.altitudeMeters = 0.0f;
            break;
        }
        case core::CommandType::Takeoff: {
            if (internalState_ == InternalState::Armed) {
                state_.mode = core::FlightMode::Takeoff;
                targetAltitude_ = 1.5f;  // Target 1.5m altitude
                state_.throttle = 0.6f;
                state_.altitudeMeters += 0.5f * dt;  // Simulated climb
                if (state_.altitudeMeters >= targetAltitude_) {
                    state_.altitudeMeters = targetAltitude_;
                    state_.mode = core::FlightMode::Hover;
                }
            }
            break;
        }
        case core::CommandType::Land: {
            state_.mode = core::FlightMode::Landing;
            state_.throttle = 0.3f;
            state_.altitudeMeters -= 0.3f * dt;  // Descent
            if (state_.altitudeMeters <= 0.0f) {
                state_.altitudeMeters = 0.0f;
                state_.throttle = 0.0f;
                state_.mode = core::FlightMode::Armed;
            }
            break;
        }
        case core::CommandType::Hold: {
            state_.mode = core::FlightMode::Hover;
            updateAltitudeHold(dt);
            break;
        }
        case core::CommandType::SetAttitude: {
            if (internalState_ == InternalState::Armed) {
                applySetAttitude(pendingCommand_, dt);
            }
            break;
        }
        default:
            break;
    }
    
    state_.heartbeat++;
    pendingCommand_.type = core::CommandType::None;
}

void FlightController::applySetAttitude(const core::VehicleCommand& cmd, float dt) {
    float rollError = cmd.roll - state_.roll;
    float pitchError = cmd.pitch - state_.pitch;
    float yawError = cmd.yaw - state_.yaw;
    
    // Normalize yaw error to [-PI, PI]
    while (yawError > 3.14159f) yawError -= 6.28318f;
    while (yawError < -3.14159f) yawError += 6.28318f;
    
    float rollCorrection = rollPid_.update(rollError, dt);
    float pitchCorrection = pitchPid_.update(pitchError, dt);
    float yawCorrection = yawPid_.update(yawError, dt);
    
    state_.roll += rollCorrection * dt;
    state_.pitch += pitchCorrection * dt;
    state_.yaw += yawCorrection * dt;
    state_.throttle = cmd.throttle;
    
    // Stay in Armed mode during attitude control
    state_.mode = core::FlightMode::Armed;
    
    // Update altitude estimation from throttle
    static float altVelocity = 0.0f;
    altVelocity += (state_.throttle - 0.5f) * 0.1f * dt;
    state_.altitudeMeters += altVelocity * dt;
    if (state_.altitudeMeters < 0.0f) {
        state_.altitudeMeters = 0.0f;
        altVelocity = 0.0f;
    }
}

void FlightController::updateAltitudeHold(float dt) {
    float altitudeError = targetAltitude_ - state_.altitudeMeters;
    float correction = altitudePid_.update(altitudeError, dt);
    
    state_.throttle = 0.5f + correction;
    if (state_.throttle > 0.8f) state_.throttle = 0.8f;
    if (state_.throttle < 0.2f) state_.throttle = 0.2f;
    
    // Simple altitude drift model
    state_.altitudeMeters += (state_.throttle - 0.5f) * 0.5f * dt;
}

void FlightController::acceptCommand(const core::VehicleCommand& cmd) {
    pendingCommand_ = cmd;
    failsafeTimer_ = 0;
}

const core::VehicleState& FlightController::state() const noexcept {
    return state_;
}

void FlightController::reset() noexcept {
    state_ = {};
    pendingCommand_ = {};
    internalState_ = InternalState::Disarmed;
    failsafeActive_ = false;
    failsafeTimer_ = 0;
    targetAltitude_ = 0.0f;
    altitudeIntegral_ = 0.0f;
    rollPid_.reset();
    pitchPid_.reset();
    yawPid_.reset();
    altitudePid_.reset();
}

bool FlightController::isArmed() const noexcept {
    return internalState_ == InternalState::Armed;
}

bool FlightController::arm() {
    if (internalState_ == InternalState::Disarmed && !failsafeActive_) {
        internalState_ = InternalState::Armed;
        state_.mode = core::FlightMode::Armed;
        rollPid_.reset();
        pitchPid_.reset();
        yawPid_.reset();
        altitudePid_.reset();
        altitudeIntegral_ = 0.0f;
        targetAltitude_ = state_.altitudeMeters;
        return true;
    }
    return false;
}

bool FlightController::disarm() {
    if (internalState_ == InternalState::Armed) {
        internalState_ = InternalState::Disarmed;
        state_.mode = core::FlightMode::Idle;
        state_.throttle = 0.0f;
        state_.altitudeMeters = 0.0f;
        return true;
    }
    return false;
}

void FlightController::triggerFailsafe() {
    failsafeActive_ = true;
    internalState_ = InternalState::Failsafe;
    state_.mode = core::FlightMode::Fault;
}

void FlightController::clearFailsafe() {
    failsafeActive_ = false;
    internalState_ = InternalState::Disarmed;
    state_.mode = core::FlightMode::Idle;
}

bool FlightController::isFailsafeActive() const noexcept {
    return failsafeActive_;
}

uint32_t FlightController::getHeartbeat() const noexcept {
    return state_.heartbeat;
}

void FlightController::setTargetAltitude(float meters) {
    targetAltitude_ = meters;
}

float FlightController::getTargetAltitude() const noexcept {
    return targetAltitude_;
}

void FlightController::applyFailsafe() {
    // Ramp throttle down over 1 second
    constexpr float rampRate = 1.0f;
    if (state_.throttle > 0.0f) {
        state_.throttle -= rampRate * 0.001f;  // dt ~= 1ms
        if (state_.throttle < 0.0f) state_.throttle = 0.0f;
    }
    state_.roll = 0.0f;
    state_.pitch = 0.0f;
    state_.yaw = 0.0f;
}

} // namespace drone::components
