#pragma once

#include "drivers/pwm_motor_driver.hpp"
#include "core/drone_types.hpp"

namespace drone::drivers {

struct MixerInputs {
    float thrust = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

class MotorMixer {
public:
    static MotorOutput mix(const MixerInputs& inputs);
};

} // namespace drone::drivers
