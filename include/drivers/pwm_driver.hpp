#pragma once

#include <cstdint>

namespace drone::drivers {

struct PwmConfig {
    uint32_t frequency = 400;        // Hz (400Hz typical for drone ESCs)
    uint32_t minPulseUs = 1000;      // 1ms = minimum throttle
    uint32_t maxPulseUs = 2000;      // 2ms = maximum throttle
    uint32_t timerChannel = 1;
};

class PwmDriver {
public:
    PwmDriver(uint32_t timerInstance = 2, uint32_t timerChannel = 1);
    ~PwmDriver() = default;
    
    bool init(const PwmConfig& config = PwmConfig{});
    void setDutyCycle(uint32_t channel, float dutyCycle01);
    void setPulseUs(uint32_t channel, uint32_t pulseUs);
    void setAllOutputs(float m1, float m2, float m3, float m4);

private:
    uint32_t timerInstance_;
    uint32_t baseChannel_;
    bool initialized_;
    
    volatile uint32_t* getBaseAddr() const;
    void enableClock();
    void configureGpioAlt(uint32_t timerInstance);
};

} // namespace drone::drivers
