#pragma once

#include <cstdint>

namespace drone::drivers {

// STM32H743 clock configuration constants
constexpr uint32_t HSE_VALUE = 8000000UL;        // 8 MHz external crystal
constexpr uint32_t CPU_FREQ_MAX = 480000000UL;   // 480 MHz max
constexpr uint32_t CPU_FREQ_TARGET = 400000000UL; // 400 MHz target
constexpr uint32_t AHB_FREQ = 200000000UL;       // 200 MHz AHB bus
constexpr uint32_t APB1_FREQ = 100000000UL;      // 100 MHz APB1
constexpr uint32_t APB2_FREQ = 100000000UL;      // 100 MHz APB2
constexpr uint32_t APB3_FREQ = 200000000UL;      // 200 MHz APB3
constexpr uint32_t APB4_FREQ = 100000000UL;      // 100 MHz APB4

class SystemClock {
public:
    enum class Result {
        Ok,
        ErrorHse,
        ErrorPll,
        ErrorLse
    };

    static Result init();
    static uint32_t getCoreClockHz();
    static void delayMs(uint32_t ms);
    static void delayUs(uint32_t us);
    static uint32_t getTickMs() { return systickCounter_; }

    // Public for ISR access
    static volatile uint32_t systickCounter_;
    static uint32_t coreClockHz_;

private:
    static void configureFlashLatency(uint32_t voltageScale);
    static bool waitForHseReady();
    static bool configurePll();
    static bool configureLse();
};

} // namespace drone::drivers

// Legacy C-compatible function for compatibility
extern "C" void SystemClock_Config(void);
extern "C" void SysTick_Handler(void);
