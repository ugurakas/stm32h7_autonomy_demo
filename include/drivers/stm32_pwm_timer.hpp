#pragma once

#include <cstdint>

#include "drivers/pwm_driver.hpp"

namespace drone::drivers {

/// Thin wrapper around the register-level PwmDriver, giving
/// Stm32PwmMotorDriver a timer type with an init()/setDutyCycle() surface.
class Stm32PwmTimer {
public:
    explicit Stm32PwmTimer(std::uint32_t timerInstance = 2);

    void init();
    void setDutyCycle(std::uint32_t channel, float dutyCycle);

private:
    PwmDriver pwm_;
};

} // namespace drone::drivers
