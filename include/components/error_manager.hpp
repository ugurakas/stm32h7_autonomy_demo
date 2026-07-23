/**
 * @file    error_manager.hpp
 * @brief   Error and failsafe management — record, auto-recovery, health monitoring.
 *
 * @details Implements a structured error management system:
 *          - 16 error slots with active/inactive tracking
 *          - Auto-recovery with configurable max attempts (default 3)
 *          - Periodic recovery attempt every 1 second
 *          - System health reporting
 *          - Most severe error identification
 *          - Error codes: IMU failure, motor failure, battery low,
 *            comm loss, GPS loss, sensor timeout, arm denied,
 *            failsafe triggered, I2C bus error, UART overrun,
 *            watchdog reset, altitude limit exceeded
 *
 * @ingroup components
 */

#pragma once

#include <cstdint>
#include "core/drone_types.hpp"

namespace drone::components {

/**
 * @brief  Error codes used across all systems.
 */
enum class ErrorCode : uint8_t {
    None = 0,
    ImuFailure,
    MotorFailure,
    BatteryLow,
    CommunicationLoss,
    GpsLoss,
    SensorTimeout,
    ArmDenied,
    FailsafeTriggered,
    I2cBusError,
    UartOverrun,
    WatchdogReset,
    AltitudeLimitExceeded
};

/// Structure representing a single error event.
struct ErrorEvent {
    ErrorCode code;              ///< Error code
    uint32_t timestampMs;        ///< First occurrence timestamp
    uint32_t recoveryAttempts;   ///< Number of recovery attempts made
    bool isActive;               ///< Whether the error is still active
};

/**
 * @defgroup error_manager Error Manager
 * @brief    Error recording, auto-recovery, and system health.
 *
 * ### Usage
 * @code{.cpp}
 *   ErrorManager em;
 *   em.reportError(ErrorCode::BatteryLow);
 *   em.setAutoRecovery(true);
 *   em.update(0.01f);
 *   if (em.hasError(ErrorCode::BatteryLow)) { ... }
 *   ErrorCode severe = em.getMostSevereError();
 * @endcode
 *
 * @{
 */

class ErrorManager {
public:
    ErrorManager();

    /** @brief  Periodic update — attempts auto-recovery if enabled. */
    void update(float dt);

    /// Report an error (increments count, sets active).
    void reportError(ErrorCode code);

    /// Clear a specific error (deactivates the slot).
    void clearError(ErrorCode code);

    /// Clear all errors.
    void clearAll();

    /// @return true if any error is active.
    bool hasError() const noexcept;

    /// @return true if the given error code is active.
    bool hasError(ErrorCode code) const noexcept;

    /// @return true if no errors are active.
    bool isSystemHealthy() const noexcept;

    /// @return Total number of active errors.
    uint32_t getErrorCount() const noexcept;

    /// @return Total occurrence count for an error code.
    uint32_t getErrorCount(ErrorCode code) const noexcept;

    /// @return The most severe active error code.
    ErrorCode getMostSevereError() const noexcept;

    // ---- Auto-recovery ----

    /// Enable or disable automatic recovery attempts.
    void setAutoRecovery(bool enable);

    /// Manually trigger one recovery attempt.
    bool attemptRecovery();

private:
    static constexpr uint32_t MAX_ERRORS           = 16;    ///< Maximum tracked errors
    static constexpr uint32_t MAX_RECOVERY_ATTEMPTS = 3;    ///< Max retries per error
    static constexpr uint32_t RECOVERY_INTERVAL_MS  = 1000; ///< Interval between attempts

    struct ErrorRecord {
        ErrorCode code = ErrorCode::None;
        uint32_t timestampMs = 0;
        uint32_t count = 0;
        uint32_t recoveryAttempts = 0;
        bool active = false;
    };

    ErrorRecord errors_[MAX_ERRORS];
    uint32_t errorCount_;
    uint32_t totalErrors_;
    bool autoRecovery_;
    uint32_t lastRecoveryTime_;

    /// Find the slot index for an error code, or -1 if full.
    int findErrorSlot(ErrorCode code);

    /// Get pointer to the record for an error code, or nullptr.
    ErrorRecord* getErrorRecord(ErrorCode code);
};

/** @} */  // end of error_manager group

} // namespace drone::components
