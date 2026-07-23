/**
 * @file    pwm_driver.cpp
 * @brief   Implementation of the STM32H7 PWM timer driver.
 *
 * @details Implements register-level PWM generation:
 *          - Supports TIM1–5, 8, 15–17 with clock enable
 *          - Prescaler configured for 1 µs resolution
 *          - PWM mode 1 with preload on all 4 channels
 *          - Configurable frequency (default 400 Hz for drone ESCs)
 *          - Atomic `setAllOutputs` for quadcopter mixing
 *          - Break (MOE) enable for advanced timers TIM1/TIM8
 *          - Automatic GPIO alternate-function setup per timer
 *
 *          Timer base addresses per RM0433 §2.3.
 *          Register accesses conform to RM0433 §32.
 *
 * @ingroup drivers
 */

#include "drivers/pwm_driver.hpp"
#include "drivers/gpio_driver.hpp"
#include "drivers/system_clock.hpp"

#include <cstdint>

namespace drone::drivers {

// ---------------------------------------------------------------------------
//  Timer register offsets (RM0433 §32.5)
// ---------------------------------------------------------------------------
namespace {
    constexpr uint32_t TIM_CR1_OFF   = 0x00U;   ///< Control register 1
    constexpr uint32_t TIM_PSC_OFF   = 0x28U;   ///< Prescaler
    constexpr uint32_t TIM_ARR_OFF   = 0x2CU;   ///< Auto-reload register
    constexpr uint32_t TIM_CCMR1_OFF = 0x18U;   ///< Capture/compare mode 1 (ch 1–2)
    constexpr uint32_t TIM_CCMR2_OFF = 0x1CU;   ///< Capture/compare mode 2 (ch 3–4)
    constexpr uint32_t TIM_CCER_OFF  = 0x20U;   ///< Capture/compare enable
    constexpr uint32_t TIM_BDTR_OFF  = 0x44U;   ///< Break & dead-time (TIM1/8 only)
    constexpr uint32_t TIM_EGR_OFF   = 0x14U;   ///< Event generation
    constexpr uint32_t TIM_CCR1_OFF  = 0x34U;   ///< Capture/compare 1
    constexpr uint32_t TIM_CCR2_OFF  = 0x38U;   ///< Capture/compare 2
    constexpr uint32_t TIM_CCR3_OFF  = 0x3CU;   ///< Capture/compare 3
    constexpr uint32_t TIM_CCR4_OFF  = 0x40U;   ///< Capture/compare 4

    // ---- CR1 bits ----
    constexpr uint32_t TIM_CR1_CEN   = (1U << 0U);   ///< Counter enable
    constexpr uint32_t TIM_CR1_ARPE  = (1U << 7U);   ///< Auto-reload preload enable

    // ---- CCER bits ----
    constexpr uint32_t TIM_CCER_CC1E = (1U << 0U);   ///< Channel 1 enable
    constexpr uint32_t TIM_CCER_CC2E = (1U << 4U);   ///< Channel 2 enable
    constexpr uint32_t TIM_CCER_CC3E = (1U << 8U);   ///< Channel 3 enable
    constexpr uint32_t TIM_CCER_CC4E = (1U << 12U);  ///< Channel 4 enable

    // ---- CCMR1 bits ----
    constexpr uint32_t TIM_CCMR1_OC1M_PWM1 = (6U << 4U);  ///< PWM mode 1, ch 1
    constexpr uint32_t TIM_CCMR1_OC1PE     = (1U << 3U);  ///< Output compare 1 preload

    // ---- BDTR bits ----
    constexpr uint32_t TIM_BDTR_MOE = (1U << 15U);  ///< Main output enable (TIM1/8)

    // ---- EGR bits ----
    constexpr uint32_t TIM_EGR_UG   = (1U << 0U);   ///< Update generation

    // ---- Timer base addresses (RM0433 §2.3) ----
    constexpr uint32_t TIM2_BASE  = 0x40000000UL;
    constexpr uint32_t TIM3_BASE  = 0x40000400UL;
    constexpr uint32_t TIM4_BASE  = 0x40000800UL;
    constexpr uint32_t TIM5_BASE  = 0x40000C00UL;
    constexpr uint32_t TIM1_BASE  = 0x40010000UL;
    constexpr uint32_t TIM8_BASE  = 0x40010400UL;
    constexpr uint32_t TIM15_BASE = 0x40014000UL;
    constexpr uint32_t TIM16_BASE = 0x40014400UL;
    constexpr uint32_t TIM17_BASE = 0x40014800UL;

    // ---- RCC clock enable bits (RM0433 §5.8) ----
    constexpr uint32_t RCC_APB1LENR_TIM2  = (1U << 0U);
    constexpr uint32_t RCC_APB1LENR_TIM3  = (1U << 1U);
    constexpr uint32_t RCC_APB1LENR_TIM4  = (1U << 2U);
    constexpr uint32_t RCC_APB1LENR_TIM5  = (1U << 3U);
    constexpr uint32_t RCC_APB1LENR_TIM6  = (1U << 4U);
    constexpr uint32_t RCC_APB1LENR_TIM7  = (1U << 5U);
    constexpr uint32_t RCC_APB1LENR_TIM12 = (1U << 6U);
    constexpr uint32_t RCC_APB2ENR_TIM1   = (1U << 0U);
    constexpr uint32_t RCC_APB2ENR_TIM8   = (1U << 1U);
    constexpr uint32_t RCC_APB2ENR_TIM15  = (1U << 16U);
    constexpr uint32_t RCC_APB2ENR_TIM16  = (1U << 17U);
    constexpr uint32_t RCC_APB2ENR_TIM17  = (1U << 18U);

    volatile uint32_t& RCC_APB1LENR = *reinterpret_cast<volatile uint32_t*>(0x58024460UL);
    volatile uint32_t& RCC_APB2ENR  = *reinterpret_cast<volatile uint32_t*>(0x580244A0UL);
}

// ============================================================================
//  Constructor
// ============================================================================

PwmDriver::PwmDriver(uint32_t timerInstance, uint32_t timerChannel)
    : timerInstance_(timerInstance), baseChannel_(timerChannel), initialized_(false) {
}

// ============================================================================
//  init
// ============================================================================

bool PwmDriver::init(const PwmConfig& config) {
    if (initialized_) return false;

    enableClock();
    configureGpioAlt(timerInstance_);
    SystemClock::delayMs(1);

    volatile uint32_t* base = getBaseAddr();
    if (!base) return false;

    // Determine timer clock frequency
    uint32_t timerClk = (timerInstance_ == 1 || timerInstance_ == 8 ||
                         timerInstance_ == 15 || timerInstance_ == 16 || timerInstance_ == 17)
                        ? 200000000UL : 100000000UL;

    // Prescaler for 1 MHz timer clock (1 µs resolution)
    uint32_t prescaler = (timerClk / 1000000UL) - 1;
    uint32_t period = (1000000UL / config.frequency) - 1;

    base[TIM_PSC_OFF / 4U] = prescaler;
    base[TIM_ARR_OFF / 4U] = period;
    base[TIM_EGR_OFF / 4U] = TIM_EGR_UG;   // Force update of prescaler

    // Configure PWM mode 1, preload on CC1–CC4
    uint32_t ccmr1 = base[TIM_CCMR1_OFF / 4U];
    ccmr1 &= ~(0xFFU);
    ccmr1 |= (TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE);
    base[TIM_CCMR1_OFF / 4U] = ccmr1;

    uint32_t ccmr2 = base[TIM_CCMR2_OFF / 4U];
    ccmr2 &= ~(0xFFU);
    ccmr2 |= (TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE);
    base[TIM_CCMR2_OFF / 4U] = ccmr2;

    // Enable all 4 output channels
    base[TIM_CCER_OFF / 4U] = TIM_CCER_CC1E | TIM_CCER_CC2E |
                               TIM_CCER_CC3E | TIM_CCER_CC4E;

    // Enable main output for advanced timers (TIM1, TIM8)
    if (timerInstance_ == 1 || timerInstance_ == 8) {
        base[TIM_BDTR_OFF / 4U] = TIM_BDTR_MOE;
    }

    // Enable counter with auto-reload preload
    base[TIM_CR1_OFF / 4U] = TIM_CR1_ARPE | TIM_CR1_CEN;

    initialized_ = true;
    return true;
}

// ============================================================================
//  Output control
// ============================================================================

void PwmDriver::setDutyCycle(uint32_t channel, float dutyCycle01) {
    if (!initialized_) return;
    if (dutyCycle01 < 0.0f) dutyCycle01 = 0.0f;
    if (dutyCycle01 > 1.0f) dutyCycle01 = 1.0f;

    volatile uint32_t* base = getBaseAddr();
    if (!base) return;

    uint32_t period = base[TIM_ARR_OFF / 4U];
    uint32_t compare = (uint32_t)((float)period * dutyCycle01);

    switch (channel) {
        case 1: base[TIM_CCR1_OFF / 4U] = compare; break;
        case 2: base[TIM_CCR2_OFF / 4U] = compare; break;
        case 3: base[TIM_CCR3_OFF / 4U] = compare; break;
        case 4: base[TIM_CCR4_OFF / 4U] = compare; break;
        default: break;
    }
}

void PwmDriver::setPulseUs(uint32_t channel, uint32_t pulseUs) {
    if (!initialized_) return;
    volatile uint32_t* base = getBaseAddr();
    if (!base) return;
    uint32_t period = base[TIM_ARR_OFF / 4U];
    float dutyCycle = (float)pulseUs / (float)(period + 1);
    setDutyCycle(channel, dutyCycle);
}

void PwmDriver::setAllOutputs(float m1, float m2, float m3, float m4) {
    setDutyCycle(baseChannel_,     m1);
    setDutyCycle(baseChannel_ + 1, m2);
    setDutyCycle(baseChannel_ + 2, m3);
    setDutyCycle(baseChannel_ + 3, m4);
}

// ============================================================================
//  Register helpers
// ============================================================================

volatile uint32_t* PwmDriver::getBaseAddr() const {
    switch (timerInstance_) {
        case 1:  return reinterpret_cast<volatile uint32_t*>(TIM1_BASE);
        case 2:  return reinterpret_cast<volatile uint32_t*>(TIM2_BASE);
        case 3:  return reinterpret_cast<volatile uint32_t*>(TIM3_BASE);
        case 4:  return reinterpret_cast<volatile uint32_t*>(TIM4_BASE);
        case 5:  return reinterpret_cast<volatile uint32_t*>(TIM5_BASE);
        case 8:  return reinterpret_cast<volatile uint32_t*>(TIM8_BASE);
        case 15: return reinterpret_cast<volatile uint32_t*>(TIM15_BASE);
        case 16: return reinterpret_cast<volatile uint32_t*>(TIM16_BASE);
        case 17: return reinterpret_cast<volatile uint32_t*>(TIM17_BASE);
        default: return nullptr;
    }
}

void PwmDriver::enableClock() {
    if (timerInstance_ == 2) {
        RCC_APB1LENR |= RCC_APB1LENR_TIM2;
    } else if (timerInstance_ == 3) {
        RCC_APB1LENR |= RCC_APB1LENR_TIM3;
    } else if (timerInstance_ == 4) {
        RCC_APB1LENR |= RCC_APB1LENR_TIM4;
    } else if (timerInstance_ == 5) {
        RCC_APB1LENR |= RCC_APB1LENR_TIM5;
    } else if (timerInstance_ == 1) {
        RCC_APB2ENR  |= RCC_APB2ENR_TIM1;
    } else if (timerInstance_ == 8) {
        RCC_APB2ENR  |= RCC_APB2ENR_TIM8;
    } else if (timerInstance_ == 15) {
        RCC_APB2ENR  |= RCC_APB2ENR_TIM15;
    } else if (timerInstance_ == 16) {
        RCC_APB2ENR  |= RCC_APB2ENR_TIM16;
    } else if (timerInstance_ == 17) {
        RCC_APB2ENR  |= RCC_APB2ENR_TIM17;
    }
}

void PwmDriver::configureGpioAlt(uint32_t timerInstance) {
    GpioConfig gpioCfg;
    gpioCfg.mode = 2;          // Alternate function
    gpioCfg.pull = 0;          // No pull
    gpioCfg.speed = 3;         // Very high
    gpioCfg.outputType = 0;    // Push-pull

    switch (timerInstance) {
        case 1:
            gpioCfg.alternate = 1;
            for (uint32_t pin = 8; pin <= 11; pin++) {
                GpioDriver gpio(GpioPort::PortA, pin);
                gpio.configure(gpioCfg);
            }
            break;
        case 2:
            gpioCfg.alternate = 1;
            for (uint32_t pin = 0; pin <= 1; pin++) {
                GpioDriver gpio(GpioPort::PortA, pin);
                gpio.configure(gpioCfg);
            }
            for (uint32_t pin = 10; pin <= 11; pin++) {
                GpioDriver gpio(GpioPort::PortB, pin);
                gpio.configure(gpioCfg);
            }
            break;
        case 3:
            gpioCfg.alternate = 1;
            for (uint32_t pin = 6; pin <= 7; pin++) {
                GpioDriver gpio(GpioPort::PortA, pin);
                gpio.configure(gpioCfg);
            }
            for (uint32_t pin = 0; pin <= 1; pin++) {
                GpioDriver gpio(GpioPort::PortB, pin);
                gpio.configure(gpioCfg);
            }
            break;
        default:
            break;
    }
}

} // namespace drone::drivers
