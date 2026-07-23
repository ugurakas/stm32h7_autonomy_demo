/**
 * @file    pwm_driver.hpp
 * @brief   STM32H7 PWM timer driver — register-level, 400 Hz for drone ESCs.
 *
 * @details Provides a generic PWM output driver:
 *          - Supports any timer (TIM1–5, 8, 15–17)
 *          - Configurable frequency (default 400 Hz for drone ESCs)
 *          - 1 µs resolution via prescaler
 *          - 4 output channels per timer
 *          - PWM mode 1 with preload enabled
 *          - Pulse-width control in µs or duty-cycle (0–1)
 *          - Convenience `setAllOutputs` for quadcopter motor mixing
 *          - Automatic GPIO alternate-function configuration
 *
 *          Hardware reference: STM32H743 RM0433 §32 (General-purpose timers).
 *          Register-level access — no HAL dependency.
 *
 * @ingroup drivers
 */

#pragma once

#include <cstdint>

namespace drone::drivers {

/**
 * @brief PWM channel configuration.
 */
struct PwmConfig {
    uint32_t frequency = 400;        ///< PWM frequency in Hz
    uint32_t minPulseUs = 1000;      ///< Minimum pulse width (µs) for ESC
    uint32_t maxPulseUs = 2000;      ///< Maximum pulse width (µs) for ESC
    uint32_t timerChannel = 1;       ///< Starting channel (1–4)
};

/**
 * @defgroup pwm_driver PWM Timer Driver
 * @brief    Register-level PWM output driver for STM32H7.
 *
 * ### Usage
 * @code{.cpp}
 *   PwmDriver pwm(2, 1);                       // TIM2, channels 1–4
 *   pwm.init({ .frequency = 400 });            // 400 Hz for ESC
 *   pwm.setDutyCycle(1, 0.5f);                 // Channel 1 @ 50%
 *   pwm.setPulseUs(1, 1500);                   // Channel 1 @ 1500 µs
 *   pwm.setAllOutputs(0.5f, 0.5f, 0.5f, 0.5f); // All motors at hover
 * @endcode
 *
 * @{
 */

class PwmDriver {
public:
    /**
     * @brief  Construct a PWM timer driver instance.
     * @param  timerInstance  Timer instance (1–5, 8, 15–17).
     * @param  timerChannel   Starting output channel (1–4).
     *         Channels are used consecutively: ch, ch+1, ch+2, ch+3.
     */
    PwmDriver(uint32_t timerInstance = 2, uint32_t timerChannel = 1);

    /// Destructor (trivial).
    ~PwmDriver() = default;

    /**
     * @brief  Initialise the timer for PWM generation.
     * @param  config  PWM configuration (frequency, pulse range).
     * @return true on success, false if already initialised.
     */
    bool init(const PwmConfig& config = PwmConfig{});

    /** @brief  Set duty cycle for a specific channel.
     *  @param  channel      Channel number (1–4).
     *  @param  dutyCycle01  Duty cycle as a float 0.0–1.0. */
    void setDutyCycle(uint32_t channel, float dutyCycle01);

    /** @brief  Set pulse width in microseconds for a channel.
     *  @param  channel  Channel number (1–4).
     *  @param  pulseUs  Pulse width in µs. */
    void setPulseUs(uint32_t channel, uint32_t pulseUs);

    /** @brief  Set all 4 outputs at once (for quadcopter motor mixing).
     *  @param  m1  Motor 1 duty cycle (0–1).
     *  @param  m2  Motor 2 duty cycle (0–1).
     *  @param  m3  Motor 3 duty cycle (0–1).
     *  @param  m4  Motor 4 duty cycle (0–1). */
    void setAllOutputs(float m1, float m2, float m3, float m4);

private:
    uint32_t timerInstance_;   ///< Timer instance number
    uint32_t baseChannel_;     ///< Starting channel
    bool initialized_;         ///< Whether the timer has been configured

    /// @return Base address of the timer registers.
    volatile uint32_t* getBaseAddr() const;

    void enableClock();
    void configureGpioAlt(uint32_t timerInstance);
};

/** @} */  // end of pwm_driver group

} // namespace drone::drivers
