#include "components/pid_controller.hpp"

namespace drone::components {

float PidController::update(float error, float dt) {
    if (dt <= 0.0f) dt = 0.001f;
    
    // Proportional term
    float pTerm = gains_.kp * error;
    
    // Integral term with anti-windup clamping
    integral_ += error * dt;
    
    // Apply integral limits
    if (integral_ > integralMax_) integral_ = integralMax_;
    else if (integral_ < integralMin_) integral_ = integralMin_;
    
    float iTerm = gains_.ki * integral_;
    
    // Derivative on measurement (avoid derivative kick)
    float dInput = (error - previousError_) / dt;
    
    // Low-pass filter for derivative term
    float alpha = dt / (dFilterTau_ + dt);
    dState_ = dState_ + alpha * (dInput - dState_);
    
    float dTerm = gains_.kd * dState_;
    
    previousError_ = error;
    
    // Calculate output
    float output = pTerm + iTerm - dTerm;
    
    // Apply output limits
    if (output > outputMax_) {
        output = outputMax_;
        // Anti-windup: stop integral growth when output is saturated
        if (iTerm > 0) integral_ -= error * dt;
    } else if (output < outputMin_) {
        output = outputMin_;
        if (iTerm < 0) integral_ -= error * dt;
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
