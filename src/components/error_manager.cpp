#include "components/error_manager.hpp"
#include "drivers/system_clock.hpp"

namespace drone::components {

ErrorManager::ErrorManager()
    : errorCount_(0), totalErrors_(0), autoRecovery_(true), lastRecoveryTime_(0) {
    clearAll();
}

void ErrorManager::update(float dt) {
    (void)dt;
    
    // Auto-recovery attempt
if (autoRecovery_ && hasError()) {
        uint32_t now = drone::drivers::SystemClock::getTickMs();
        if (now - lastRecoveryTime_ >= RECOVERY_INTERVAL_MS) {
            attemptRecovery();
            lastRecoveryTime_ = now;
        }
    }
}

void ErrorManager::reportError(ErrorCode code) {
    totalErrors_++;
    
    ErrorRecord* record = getErrorRecord(code);
    if (record) {
        record->count++;
        record->active = true;
record->timestampMs = drone::drivers::SystemClock::getTickMs();
        return;
    }
    
    // Find empty slot
    int slot = findErrorSlot(ErrorCode::None);
    if (slot >= 0) {
        errors_[slot].code = code;
        errors_[slot].timestampMs = drone::drivers::SystemClock::getTickMs();
        errors_[slot].count = 1;
        errors_[slot].recoveryAttempts = 0;
        errors_[slot].active = true;
        errorCount_++;
    }
}

void ErrorManager::clearError(ErrorCode code) {
    ErrorRecord* record = getErrorRecord(code);
    if (record) {
        record->active = false;
        errorCount_--;
        if (errorCount_ < 0) errorCount_ = 0;
    }
}

void ErrorManager::clearAll() {
    for (uint32_t i = 0; i < MAX_ERRORS; ++i) {
        errors_[i].code = ErrorCode::None;
        errors_[i].timestampMs = 0;
        errors_[i].count = 0;
        errors_[i].recoveryAttempts = 0;
        errors_[i].active = false;
    }
    errorCount_ = 0;
}

bool ErrorManager::hasError() const noexcept {
    return errorCount_ > 0;
}

bool ErrorManager::hasError(ErrorCode code) const noexcept {
    const ErrorRecord* record = nullptr;
    for (uint32_t i = 0; i < MAX_ERRORS; ++i) {
        if (errors_[i].code == code) {
            record = &errors_[i];
            break;
        }
    }
    return record && record->active;
}

bool ErrorManager::isSystemHealthy() const noexcept {
    if (errorCount_ > 0) return false;
    return true;
}

uint32_t ErrorManager::getErrorCount() const noexcept {
    return errorCount_;
}

uint32_t ErrorManager::getErrorCount(ErrorCode code) const noexcept {
    const ErrorRecord* record = nullptr;
    for (uint32_t i = 0; i < MAX_ERRORS; ++i) {
        if (errors_[i].code == code) {
            record = &errors_[i];
            break;
        }
    }
    return record ? record->count : 0;
}

ErrorCode ErrorManager::getMostSevereError() const noexcept {
    for (uint32_t i = 0; i < MAX_ERRORS; ++i) {
        if (errors_[i].active) {
            return errors_[i].code;
        }
    }
    return ErrorCode::None;
}

void ErrorManager::setAutoRecovery(bool enable) {
    autoRecovery_ = enable;
}

bool ErrorManager::attemptRecovery() {
    for (uint32_t i = 0; i < MAX_ERRORS; ++i) {
        if (errors_[i].active) {
            if (errors_[i].recoveryAttempts < MAX_RECOVERY_ATTEMPTS) {
                errors_[i].recoveryAttempts++;
                return true;
            } else {
                // Max attempts reached, keep error active
                return false;
            }
        }
    }
    return true;
}

int ErrorManager::findErrorSlot(ErrorCode code) {
    for (uint32_t i = 0; i < MAX_ERRORS; ++i) {
        if (errors_[i].code == code) {
            return i;
        }
    }
    return -1;
}

ErrorManager::ErrorRecord* ErrorManager::getErrorRecord(ErrorCode code) {
    for (uint32_t i = 0; i < MAX_ERRORS; ++i) {
        if (errors_[i].code == code) {
            return &errors_[i];
        }
    }
    return nullptr;
}

} // namespace drone::components
