#include "components/led_status.hpp"
#include "drivers/gpio_driver.hpp"
#include "drivers/system_clock.hpp"

namespace drone::components {

using namespace drone::drivers;

LedStatus::LedStatus()
    : ledRed_(GpioPort::PortB, 14),
      ledGreen_(GpioPort::PortB, 0),
      ledBlue_(GpioPort::PortB, 7),
      currentPattern_(LedPattern::Off),
      timer_(0.0f), blinkState_(false), blinkCount_(0) {
}

void LedStatus::init() {
    GpioConfig cfg;
    cfg.mode = 1;
    cfg.pull = 0;
    cfg.speed = 2;
    cfg.outputType = 0;
    
    ledRed_.configure(cfg);
    ledGreen_.configure(cfg);
    ledBlue_.configure(cfg);
    
    setLed(false, false, false);
}

void LedStatus::update(float dt) {
    timer_ += dt;
    updatePattern(dt);
}

void LedStatus::setPattern(LedPattern pattern) {
    if (currentPattern_ != pattern) {
        currentPattern_ = pattern;
        timer_ = 0.0f;
        blinkCount_ = 0;
        blinkState_ = false;
    }
}

void LedStatus::setColor(float r, float g, float b) {
    setLed(r > 0.5f, g > 0.5f, b > 0.5f);
}

void LedStatus::setSystemState(bool armed, bool fault, bool autonomous, bool batteryLow) {
    if (fault) {
        setPattern(LedPattern::FastBlink);
        setLed(true, false, false);
        return;
    }
    if (batteryLow) {
        setPattern(LedPattern::SlowBlink);
        setLed(true, true, false);
        return;
    }
    if (armed && autonomous) {
        setPattern(LedPattern::SlowBlink);
        setLed(false, false, true);
        return;
    }
    if (armed) {
        setPattern(LedPattern::SolidOn);
        setLed(false, true, false);
        return;
    }
    setPattern(LedPattern::Heartbeat);
    setLed(false, true, false);
}

LedPattern LedStatus::getPattern() const noexcept {
    return currentPattern_;
}

void LedStatus::setLed(bool red, bool green, bool blue) {
    ledRed_.write(red ? 1 : 0);
    ledGreen_.write(green ? 1 : 0);
    ledBlue_.write(blue ? 1 : 0);
}

void LedStatus::updatePattern(float dt) {
    (void)dt;
    switch (currentPattern_) {
        case LedPattern::Off:
            setLed(false, false, false);
            break;
        case LedPattern::SolidOn:
            break;
        case LedPattern::SlowBlink: {
            if (timer_ >= SLOW_BLINK_PERIOD) {
                timer_ = 0.0f;
                blinkState_ = !blinkState_;
                setLed(blinkState_, blinkState_, blinkState_);
            }
            break;
        }
        case LedPattern::FastBlink: {
            if (timer_ >= FAST_BLINK_PERIOD) {
                timer_ = 0.0f;
                blinkState_ = !blinkState_;
                setLed(blinkState_, false, false);
            }
            break;
        }
        case LedPattern::DoubleBlink: {
            if (blinkCount_ < 2) {
                if (timer_ >= DOUBLE_BLINK_PERIOD) {
                    timer_ = 0.0f;
                    blinkState_ = !blinkState_;
                    setLed(blinkState_, false, false);
                    if (!blinkState_) blinkCount_++;
                }
            } else {
                if (timer_ >= 0.5f) {
                    timer_ = 0.0f;
                    blinkCount_ = 0;
                }
            }
            break;
        }
        case LedPattern::TripleBlink: {
            if (blinkCount_ < 3) {
                if (timer_ >= TRIPLE_BLINK_PERIOD) {
                    timer_ = 0.0f;
                    blinkState_ = !blinkState_;
                    setLed(blinkState_, false, false);
                    if (!blinkState_) blinkCount_++;
                }
            } else {
                if (timer_ >= 0.5f) {
                    timer_ = 0.0f;
                    blinkCount_ = 0;
                }
            }
            break;
        }
        case LedPattern::Heartbeat: {
            float period = 1.0f;
            float pos = timer_ / period;
            if (pos < 0.1f) {
                blinkState_ = true;
            } else if (pos < 0.2f) {
                blinkState_ = false;
            } else if (pos < 0.3f) {
                blinkState_ = true;
            } else {
                blinkState_ = false;
                if (pos > 1.0f) timer_ = 0.0f;
            }
            setLed(false, blinkState_, false);
            break;
        }
        case LedPattern::FadeInOut:
        default:
            setLed(false, false, false);
            break;
    }
}

} // namespace drone::components
