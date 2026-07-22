#include "drivers/pwm_motor_driver.hpp"

#include "drivers/stm32_pwm_timer.hpp"

namespace drone::drivers {

// Initialize the simulated motor driver.
void MockPwmMotorDriver::init() {
}

void MockPwmMotorDriver::setOutputs(const MotorOutput& output) {
    lastOutput_ = output;
}

const MotorOutput& MockPwmMotorDriver::lastOutput() const noexcept {
    return lastOutput_;
}

Stm32PwmMotorDriver::Stm32PwmMotorDriver(std::uint32_t timerChannel)
    : timerChannel_(timerChannel) {
}

void Stm32PwmMotorDriver::init() {
    Stm32PwmTimer timer;
    timer.init();
    (void)timer;
}

void Stm32PwmMotorDriver::setOutputs(const MotorOutput& output) {
    Stm32PwmTimer timer;
    timer.init();
    timer.setDutyCycle(timerChannel_ + 0, output.frontLeft);
    timer.setDutyCycle(timerChannel_ + 1, output.frontRight);
    timer.setDutyCycle(timerChannel_ + 2, output.rearLeft);
    timer.setDutyCycle(timerChannel_ + 3, output.rearRight);
}

} // namespace drone::drivers
