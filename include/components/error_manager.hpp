#pragma once

#include <cstdint>
#include "core/drone_types.hpp"

namespace drone::components {

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

struct ErrorEvent {
    ErrorCode code;
    uint32_t timestampMs;
    uint32_t recoveryAttempts;
    bool isActive;
};

class ErrorManager {
public:
    ErrorManager();
    
    void update(float dt);
    void reportError(ErrorCode code);
    void clearError(ErrorCode code);
    void clearAll();
    
    bool hasError() const noexcept;
    bool hasError(ErrorCode code) const noexcept;
    bool isSystemHealthy() const noexcept;
    
    uint32_t getErrorCount() const noexcept;
    uint32_t getErrorCount(ErrorCode code) const noexcept;
    ErrorCode getMostSevereError() const noexcept;
    
    // Auto-recovery
    void setAutoRecovery(bool enable);
    bool attemptRecovery();

private:
    static constexpr uint32_t MAX_ERRORS = 16;
    static constexpr uint32_t MAX_RECOVERY_ATTEMPTS = 3;
    static constexpr uint32_t RECOVERY_INTERVAL_MS = 1000;
    
    struct ErrorRecord {
        ErrorCode code;
        uint32_t timestampMs;
        uint32_t count;
        uint32_t recoveryAttempts;
        bool active;
    };
    
    ErrorRecord errors_[MAX_ERRORS];
    uint32_t errorCount_;
    uint32_t totalErrors_;
    bool autoRecovery_;
    uint32_t lastRecoveryTime_;
    
    int findErrorSlot(ErrorCode code);
    ErrorRecord* getErrorRecord(ErrorCode code);
};

} // namespace drone::components
