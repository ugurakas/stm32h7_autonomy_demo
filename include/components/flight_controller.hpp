/**
 * @file    flight_controller.hpp
 * @brief   Drone flight controller — arm/disarm state machine, PID control, failsafe.
 *
 * @details Implements the core flight control logic:
 *          - Arm/disarm state machine with failsafe mode
 *          - Four independent PID controllers (roll, pitch, yaw, altitude)
 *          - Command processing: Arm, Disarm, Takeoff, Land, Hold, SetAttitude
 *          - Smooth altitude transitions with target tracking
 *          - Yaw error normalisation to [-π, +π]
 *          - Failsafe detection and throttle ramp-down
 *          - Altitude hold using dedicated PID controller
 *
 *          The controller operates in three internal states:
 *          `Disarmed` (zero outputs), `Armed` (normal operation),
 *          and `Failsafe` (throttle ramp-down).
 *
 * @ingroup components
 */

#pragma once

#include "components/pid_controller.hpp"
#include "core/drone_types.hpp"

namespace drone::components {

/**
 * @defgroup flight_controller Flight Controller
 * @brief    Core flight control with state machine and quadruple PID.
 *
 * ### Usage
 * @code{.cpp}
 *   FlightController fc;
 *   fc.arm();                                              // Arm the controller
 *   fc.acceptCommand({ CommandType::Takeoff });
 *   fc.update(0.01f);                                      // 10 ms tick
 *   const auto& state = fc.state();                        // Current vehicle state
 *   fc.triggerFailsafe();                                  // Emergency ramp-down
 * @endcode
 *
 * @{
 */

class FlightController {
public:
    FlightController();

    /** @brief  Main update tick — process commands and compute outputs.
     *  @param  dt  Timestep in seconds. */
    void update(float dt);

    /** @brief  Accept a vehicle command for execution on next update. */
    void acceptCommand(const core::VehicleCommand& cmd);

    /** @brief  Get the current vehicle state (read-only). */
    const core::VehicleState& state() const noexcept;

    /** @brief  Reset all state to initial (disarmed, zero). */
    void reset() noexcept;

    // ---- Arm / Disarm ----
    bool isArmed() const noexcept;
    bool arm();
    bool disarm();

    // ---- Failsafe ----
    void triggerFailsafe();
    void clearFailsafe();
    bool isFailsafeActive() const noexcept;
    uint32_t getHeartbeat() const noexcept;

    // ---- Altitude hold ----
    void setTargetAltitude(float meters);
    float getTargetAltitude() const noexcept;

private:
    enum class InternalState : uint8_t {
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

    // PID controllers
    PidController rollPid_{PidController::Gains{1.2f, 0.05f, 0.1f}};
    PidController pitchPid_{PidController::Gains{1.2f, 0.05f, 0.1f}};
    PidController yawPid_{PidController::Gains{0.8f, 0.02f, 0.05f}};
    PidController altitudePid_{PidController::Gains{0.5f, 0.01f, 0.0f}};

    void applyFailsafe();
    void applySetAttitude(const core::VehicleCommand& cmd, float dt);
    void updateAltitudeHold(float dt);
};

/** @} */  // end of flight_controller group

} // namespace drone::components
