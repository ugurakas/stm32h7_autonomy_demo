/**
 * @file    system_clock.hpp
 * @brief   STM32H7 system clock driver — register-level PLL/SysTick configuration.
 *
 * @details Provides:
 *          - Full clock tree initialisation: HSE → PLL1 → 400 MHz CPU core
 *          - SysTick-based millisecond timing (getTickMs, delayMs)
 *          - Calibrated NOP-loop microsecond delay (delayUs)
 *          - Flash wait-state and voltage-scaling management
 *
 *          Hardware reference: STM32H743 Nucleo‑144 (RM0433 Rev 7)
 *          - HSE = 8 MHz external crystal
 *          - PLL1: M=4, N=200, P=1 → 400 MHz
 *          - VOS0, 4 wait-states, D/I‑Cache enabled
 *
 * @ingroup drivers
 */

#pragma once

#include <cstdint>

namespace drone::drivers {

// =============================================================================
//  Clock-tree frequency constants (RM0433 §5.3.3)
// =============================================================================

constexpr uint32_t HSE_VALUE       = 8000000UL;     ///< 8 MHz external crystal on Nucleo‑H743ZI
constexpr uint32_t CPU_FREQ_MAX    = 480000000UL;   ///< Absolute maximum CPU frequency (Hz)
constexpr uint32_t CPU_FREQ_TARGET = 400000000UL;   ///< Target CPU frequency after PLL lock (Hz)
constexpr uint32_t AHB_FREQ        = 200000000UL;   ///< AHB bus frequency (Hz)
constexpr uint32_t APB1_FREQ       = 100000000UL;   ///< APB1 timer/peripheral bus (Hz)
constexpr uint32_t APB2_FREQ       = 100000000UL;   ///< APB2 timer/peripheral bus (Hz)
constexpr uint32_t APB3_FREQ       = 200000000UL;   ///< APB3 bus (D1 domain) frequency (Hz)
constexpr uint32_t APB4_FREQ       = 100000000UL;   ///< APB4 bus frequency (Hz)

/**
 * @defgroup  system_clock  System Clock Driver
 * @brief     Static API for STM32H7 clock tree configuration.
 *
 * ### Usage
 * @code{.cpp}
 *   SystemClock::init();               // Configure HSE → PLL → 400 MHz
 *   uint32_t now = SystemClock::getTickMs();  // Millisecond timestamp
 *   SystemClock::delayMs(100);         // Block 100 ms
 *   SystemClock::delayUs(50);          // Block 50 µs
 * @endcode
 *
 * ### Sequence (SystemClock::init)
 * 1. Power-on HSE oscillator, wait for ready
 * 2. Set voltage regulator to VOS0 (high-performance)
 * 3. Configure Flash: 4 wait-states, D/I‑Cache, WRHIGHFREQ
 * 4. Configure PLL1: source=HSE, M=4, N=200, P=1, Q=2, R=2
 * 5. Wait for PLL1 lock
 * 6. Set bus prescalers (AHB, APB1, APB2, APB3, APB4)
 * 7. Switch system clock source to PLL1
 * 8. Start SysTick with 1 ms period
 *
 * @{
 */

class SystemClock {
public:
    /// Clock initialisation results.
    enum class Result {
        Ok,          ///< Clock configured successfully.
        ErrorHse,    ///< HSE oscillator failed to start or stabilise.
        ErrorPll,    ///< PLL1 failed to lock.
        ErrorLse     ///< LSE oscillator failed (used for RTC backup domain).
    };

    /** @brief  Initialise the full clock tree (HSE → PLL1 → 400 MHz).
     *  @return Result::Ok on success, or an error code. */
    static Result init();

    /** @return The current core (CPU) clock frequency in Hz. */
    static uint32_t getCoreClockHz();

    /**
     * @brief  Blocking delay using the SysTick counter.
     * @param  ms  Delay duration in milliseconds.
     * @note   This is an IDLE-loop; interrupts will wake the CPU.
     */
    static void delayMs(uint32_t ms);

    /**
     * @brief  Blocking delay using a NOP loop (calibrated for 400 MHz).
     * @param  us  Delay duration in microseconds.
     * @note   Busy-wait.  Interrupts are NOT blocked, but CPU stays awake.
     */
    static void delayUs(uint32_t us);

    /** @return The current SysTick counter value (milliseconds since boot). */
    static uint32_t getTickMs() { return systickCounter_; }

    // -------------------------------------------------------------------------
    //  Public data – required by SysTick_Handler ISR
    // -------------------------------------------------------------------------
    static volatile uint32_t systickCounter_;   ///< Monotonic millisecond counter, incremented by SysTick ISR.
    static uint32_t coreClockHz_;               ///< Cached core clock frequency after init().

private:
    static void configureFlashLatency(uint32_t voltageScale);
    static bool waitForHseReady();
    static bool configurePll();
    static bool configureLse();
};

/** @} */  // end of system_clock group

} // namespace drone::drivers

// ---------------------------------------------------------------------------
//  C‑compatible wrappers (used by startup assembly and interrupt vectors)
// ---------------------------------------------------------------------------
extern "C" void SystemClock_Config(void);   ///< Legacy C wrapper for SystemClock::init()
extern "C" void SysTick_Handler(void);      ///< SysTick interrupt handler – increments systickCounter_
