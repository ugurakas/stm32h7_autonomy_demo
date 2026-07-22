#include "drivers/motor_mixer.hpp"

namespace drone::drivers {

namespace {

float clamp01(float value) {
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

} // namespace

MotorOutput MotorMixer::mix(const MixerInputs& inputs) {
    MotorOutput out{};

    const float normalizedThrust = clamp01(inputs.thrust);
    const float normalizedRoll = inputs.roll < -1.0f ? -1.0f : inputs.roll > 1.0f ? 1.0f : inputs.roll;
    const float normalizedPitch = inputs.pitch < -1.0f ? -1.0f : inputs.pitch > 1.0f ? 1.0f : inputs.pitch;
    const float normalizedYaw = inputs.yaw < -1.0f ? -1.0f : inputs.yaw > 1.0f ? 1.0f : inputs.yaw;

    out.frontLeft = normalizedThrust + 0.35f * normalizedRoll + 0.35f * normalizedPitch - 0.25f * normalizedYaw;
    out.frontRight = normalizedThrust - 0.35f * normalizedRoll + 0.35f * normalizedPitch + 0.25f * normalizedYaw;
    out.rearLeft = normalizedThrust - 0.35f * normalizedRoll - 0.35f * normalizedPitch - 0.25f * normalizedYaw;
    out.rearRight = normalizedThrust + 0.35f * normalizedRoll - 0.35f * normalizedPitch + 0.25f * normalizedYaw;

    out.frontLeft = clamp01(out.frontLeft);
    out.frontRight = clamp01(out.frontRight);
    out.rearLeft = clamp01(out.rearLeft);
    out.rearRight = clamp01(out.rearRight);

    return out;
}

} // namespace drone::drivers
