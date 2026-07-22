#pragma once

#include "components/pid_controller.hpp"
#include "core/drone_types.hpp"

namespace drone::components {

class FlightController {
public:
    FlightController();
    
    void update(float dt);
    void acceptCommand(const core::VehicleCommand& cmd);
    const core::VehicleState& state() const noexcept;
    void reset() noexcept;
    
    // Arm/Disarm
    bool isArmed() const noexcept;
    bool arm();
    bool disarm();
    
    // Failsafe
    void triggerFailsafe();
    void clearFailsafe();
    bool isFailsafeActive() const noexcept;
    uint32_t getHeartbeat() const noexcept;
    
    // Altitude hold
    void setTargetAltitude(float meters);
    float getTargetAltitude() const noexcept;

private:
    enum class InternalState {
        Disarmed,
        Armed,
        Failsafe
    };
    
    core::VehicleState state_{};
    core::VehicleCommand pendingCommand_{};
    InternalState internalState_;
    bool failsafeActive_;
    uint32_t failsafeTimer_;
    float targetAltitude_;
    float altitudeIntegral_;
    
    PidController rollPid_{PidController::Gains{1.2f, 0.05f, 0.1f}};
    PidController pitchPid_{PidController::Gains{1.2f, 0.05f, 0.1f}};
    PidController yawPid_{PidController::Gains{0.8f, 0.02f, 0.05f}};
    PidController altitudePid_{PidController::Gains{0.5f, 0.01f, 0.0f}};
    
    void applyFailsafe();
    void applySetAttitude(const core::VehicleCommand& cmd, float dt);
    void updateAltitudeHold(float dt);
};

} // namespace drone::components
