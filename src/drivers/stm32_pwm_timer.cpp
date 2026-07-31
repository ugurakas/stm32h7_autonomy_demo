#include "drivers/stm32_pwm_timer.hpp"

namespace drone::drivers {

Stm32PwmTimer::Stm32PwmTimer(std::uint32_t timerInstance)
    : pwm_(timerInstance, 1) {
}

void Stm32PwmTimer::init() {
    pwm_.init();
}

void Stm32PwmTimer::setDutyCycle(std::uint32_t channel, float dutyCycle) {
    pwm_.setDutyCycle(channel, dutyCycle);
}

} // namespace drone::drivers
