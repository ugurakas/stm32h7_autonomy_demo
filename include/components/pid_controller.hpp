#pragma once

namespace drone::components {

class PidController {
public:
    struct Gains {
        float kp;
        float ki;
        float kd;
        
        Gains() : kp(0.0f), ki(0.0f), kd(0.0f) {}
        Gains(float p, float i, float d) : kp(p), ki(i), kd(d) {}
    };

    PidController() = default;
    explicit PidController(const Gains& gains) : gains_(gains) {}

    float update(float error, float dt);
    void reset() noexcept;
    
    // Advanced features
    void setOutputLimits(float min, float max);
    void setIntegralLimits(float min, float max);
    void setDerivativeFilter(float tau);
    void setGains(const Gains& gains);
    const Gains& getGains() const noexcept;
    float proportional() const noexcept;
    float integral() const noexcept;
    float derivative() const noexcept;

private:
    Gains gains_{};
    float integral_ = 0.0f;
    float previousError_ = 0.0f;
    float previousMeasurement_ = 0.0f;
    float lastOutput_ = 0.0f;
    float outputMin_ = -1.0f;
    float outputMax_ = 1.0f;
    float integralMin_ = -0.5f;
    float integralMax_ = 0.5f;
    float dFilterTau_ = 0.02f;
    float dState_ = 0.0f;
};

} // namespace drone::components
