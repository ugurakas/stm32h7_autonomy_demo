/**
 * @file    led_status.hpp
 * @brief   RGB LED status indicator — patterns based on system state.
 *
 * @details Controls three GPIO-driven LEDs (Red, Green, Blue) to provide
 *          visual feedback on system state:
 *          - Pre-defined patterns: Off, Solid, Slow/Fast Blink,
 *            Double/Triple Blink, FadeInOut, Heartbeat
 *          - Auto-pattern selection based on armed/fault/autonomous/battery state
 *          - RGB colour support
 *          - Timer-based pattern sequencing
 *
 *          Hardware: PB14 (Red), PB0 (Green), PB7 (Blue) on Nucleo-H743ZI.
 *
 * @ingroup components
 */

#pragma once

#include <cstdint>
#include "drivers/gpio_driver.hpp"

namespace drone::components {

/**
 * @brief  Available LED display patterns.
 */
enum class LedPattern : uint8_t {
    Off,            ///< All LEDs off
    SolidOn,        ///< Solid colour
    SlowBlink,      ///< 1 Hz blink
    FastBlink,      ///< 5 Hz blink
    DoubleBlink,    ///< 2 fast + pause
    TripleBlink,    ///< 3 fast + pause
    FadeInOut,      ///< (reserved)
    Heartbeat       ///< Quick double pulse
};

/**
 * @defgroup led_status LED Status Indicator
 * @brief    RGB LED patterns driven by system state.
 *
 * ### Auto-pattern Mapping
 * | State | Pattern |
 * |-------|---------|
 * | Armed | Solid green |
 * | Fault | Fast blink red |
 * | Autonomous | Slow blink blue |
 * | Battery low | Fast blink red + blue |
 *
 * @{
 */

class LedStatus {
public:
    LedStatus();

    /** @brief  Configure all three LED pins as outputs. */
    void init();

    /** @brief  Update LED state — call at ~1 kHz.
     *  @param  dt  Timestep in seconds. */
    void update(float dt);

    /** @brief  Set a specific pattern.
     *  @param  pattern  The pattern to play. */
    void setPattern(LedPattern pattern);

    /** @brief  Set the RGB colour.
     *  @param  r  Red intensity (0=off, 1=on).
     *  @param  g  Green intensity.
     *  @param  b  Blue intensity. */
    void setColor(float r, float g, float b);

    /** @brief  Auto-select pattern based on system state flags.
     *  @param  armed       Whether motors are armed.
     *  @param  fault       Whether a fault is active.
     *  @param  autonomous  Whether autonomous mode is active.
     *  @param  batteryLow  Whether battery is low. */
    void setSystemState(bool armed, bool fault, bool autonomous, bool batteryLow);

    /// @return Currently active pattern.
    LedPattern getPattern() const noexcept;

private:
    drivers::GpioDriver ledRed_;    ///< Red LED pin (PB14)
    drivers::GpioDriver ledGreen_;  ///< Green LED pin (PB0)
    drivers::GpioDriver ledBlue_;   ///< Blue LED pin (PB7)

    LedPattern currentPattern_;
    float timer_;
    bool blinkState_;
    uint32_t blinkCount_;

    static constexpr float SLOW_BLINK_PERIOD   = 0.5f;    ///< 1 Hz
    static constexpr float FAST_BLINK_PERIOD   = 0.1f;    ///< 5 Hz
    static constexpr float DOUBLE_BLINK_PERIOD = 0.08f;   ///< ~12 Hz burst
    static constexpr float TRIPLE_BLINK_PERIOD = 0.06f;   ///< ~16 Hz burst

    void setLed(bool red, bool green, bool blue);
    void updatePattern(float dt);
};

/** @} */  // end of led_status group

} // namespace drone::components
