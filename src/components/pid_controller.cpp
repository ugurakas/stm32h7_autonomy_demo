/**
 * @file    pid_controller.cpp
 * @brief   Implementation of a PID controller with anti-windup.
 *
 * @details Implements a fully-featured PID controller:
 *          - Proportional, integral, derivative terms
 *          - Derivative-on-measurement (avoids derivative kick on setpoint change)
 *          - Low-pass filter on the D-term (configurable tau)
 *          - Integral anti-windup with both clamping and back-calculation
 *          - Configurable output and integral limits
 *          - Accessors for individual term contributions
 *
 *          The anti-windup strategy uses conditional integration:
 *          when the output saturates and the integral term is pushing
 *          in the same direction, integral accumulation is suspended.
 *
 * @ingroup components
 */

#include "components/pid_controller.hpp"

namespace drone::components {

float PidController::update(float error, float dt) {
    if (dt <= 0.0f) dt = 0.001f;   // Guard against zero/negative dt

    // ---- Proportional term ----
    float pTerm = gains_.kp * error;

    // ---- Integral term with clamping ----
    integral_ += error * dt;
    if (integral_ > integralMax_)      integral_ = integralMax_;
    else if (integral_ < integralMin_) integral_ = integralMin_;
    float iTerm = gains_.ki * integral_;

    // ---- Derivative term (on measurement, with low-pass filter) ----
    float dInput = (error - previousError_) / dt;
    float alpha = dt / (dFilterTau_ + dt);
    dState_ = dState_ + alpha * (dInput - dState_);
    float dTerm = gains_.kd * dState_;

    previousError_ = error;

    // ---- Output calculation ----
    float output = pTerm + iTerm - dTerm;

    // ---- Anti-windup back-calculation on saturation ----
    if (output > outputMax_) {
        output = outputMax_;
        if (gains_.ki * integral_ > 0.0f) {
            integral_ -= error * dt;   // Undo the integration
        }
    } else if (output < outputMin_) {
        output = outputMin_;
        if (gains_.ki * integral_ < 0.0f) {
            integral_ -= error * dt;
        }
    }

    lastOutput_ = output;
    return output;
}

void PidController::reset() noexcept {
    integral_ = 0.0f;
    previousError_ = 0.0f;
    previousMeasurement_ = 0.0f;
    dState_ = 0.0f;
    lastOutput_ = 0.0f;
}

void PidController::setOutputLimits(float min, float max) {
    outputMin_ = min;
    outputMax_ = max;
}

void PidController::setIntegralLimits(float min, float max) {
    integralMin_ = min;
    integralMax_ = max;
}

void PidController::setDerivativeFilter(float tau) {
    dFilterTau_ = (tau > 0.0f) ? tau : 0.001f;
}

void PidController::setGains(const Gains& gains) {
    gains_ = gains;
}

const PidController::Gains& PidController::getGains() const noexcept {
    return gains_;
}

float PidController::proportional() const noexcept {
    return gains_.kp * previousError_;
}

float PidController::integral() const noexcept {
    return gains_.ki * integral_;
}

float PidController::derivative() const noexcept {
    return gains_.kd * dState_;
}

} // namespace drone::components
