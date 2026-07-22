#pragma once

#include <cstdint>
#include "drivers/gpio_driver.hpp"

namespace drone::components {

enum class LedPattern : uint8_t {
    Off,
    SolidOn,
    SlowBlink,     // 1Hz
    FastBlink,     // 5Hz
    DoubleBlink,   // 2 fast + pause
    TripleBlink,   // 3 fast + pause
    FadeInOut,
    Heartbeat      // Quick double pulse
};

class LedStatus {
public:
    LedStatus();
    
    void init();
    void update(float dt);
    
    void setPattern(LedPattern pattern);
    void setColor(float r, float g, float b);
    
    // Auto-pattern based on system state
    void setSystemState(bool armed, bool fault, bool autonomous, bool batteryLow);
    
    LedPattern getPattern() const noexcept;

private:
    drivers::GpioDriver ledRed_;
    drivers::GpioDriver ledGreen_;
    drivers::GpioDriver ledBlue_;
    
    LedPattern currentPattern_;
    float timer_;
    bool blinkState_;
    uint32_t blinkCount_;
    
    static constexpr float SLOW_BLINK_PERIOD = 0.5f;
    static constexpr float FAST_BLINK_PERIOD = 0.1f;
    static constexpr float DOUBLE_BLINK_PERIOD = 0.08f;
    static constexpr float TRIPLE_BLINK_PERIOD = 0.06f;
    
    void setLed(bool red, bool green, bool blue);
    void updatePattern(float dt);
};

} // namespace drone::components
