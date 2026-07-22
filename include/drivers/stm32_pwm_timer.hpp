#pragma once

#include <cstdint>

namespace drone::drivers {

class Stm32PwmTimer {
public:
    void init();
    void setDutyCycle(std::uint32_t channel, float dutyCycle);
};

} // namespace drone::drivers
