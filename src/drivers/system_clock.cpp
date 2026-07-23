/**
 * @file    system_clock.cpp
 * @brief   Implementation of the STM32H7 system clock driver.
 *
 * @details Implements the full clock tree initialisation sequence:
 *          - HSE oscillator startup with timeout
 *          - PLL1 configuration: M=4, N=200, P=1 → 400 MHz from 8 MHz HSE
 *          - Voltage scaling (VOS0) and Flash 4 wait-states with D/I-Cache
 *          - Bus prescalers: AHB=200 MHz, APB1/2/4=100 MHz, APB3=200 MHz
 *          - SysTick 1 ms period interrupt for millisecond timing
 *
 *          All register addresses and bit definitions conform to
 *          RM0433 Rev 7 (STM32H743 reference manual).
 *
 * @ingroup drivers
 */

#include "drivers/system_clock.hpp"

#include <cstdint>

namespace drone::drivers {

// ---------------------------------------------------------------------------
//  Static data
// ---------------------------------------------------------------------------

/// Monotonic millisecond counter, incremented by SysTick_Handler.
volatile uint32_t SystemClock::systickCounter_ = 0;

/// Cached core clock frequency after successful init().
uint32_t SystemClock::coreClockHz_ = 0;

// ---------------------------------------------------------------------------
//  Hardware register map — RM0433 §2.3 (memory map) & §5 (RCC)
// ---------------------------------------------------------------------------
namespace {

    // ---- Flash controller (RM0433 §6) ----
    volatile uint32_t& FLASH_ACR = *reinterpret_cast<volatile uint32_t*>(0x52002000UL);

    // ---- Reset and Clock Control (RM0433 §5.8) ----
    volatile uint32_t& RCC_CR       = *reinterpret_cast<volatile uint32_t*>(0x58024400UL);
    volatile uint32_t& RCC_CFGR     = *reinterpret_cast<volatile uint32_t*>(0x58024408UL);
    volatile uint32_t& RCC_PLLCKSELR = *reinterpret_cast<volatile uint32_t*>(0x5802440CUL);
    volatile uint32_t& RCC_PLLCFGR  = *reinterpret_cast<volatile uint32_t*>(0x58024410UL);
    volatile uint32_t& RCC_PLL1DIVR = *reinterpret_cast<volatile uint32_t*>(0x58024414UL);
    volatile uint32_t& RCC_D1CFGR   = *reinterpret_cast<volatile uint32_t*>(0x58024418UL);
    volatile uint32_t& RCC_D2CFGR   = *reinterpret_cast<volatile uint32_t*>(0x5802441CUL);
    volatile uint32_t& RCC_D3CFGR   = *reinterpret_cast<volatile uint32_t*>(0x58024420UL);
    volatile uint32_t& RCC_BDCR     = *reinterpret_cast<volatile uint32_t*>(0x580244A0UL);

    // ---- Power controller (RM0433 §8) ----
    volatile uint32_t& PWR_D3CR     = *reinterpret_cast<volatile uint32_t*>(0x58024808UL);

    // ---- System control block – SysTick (ARM® v7‑M) ----
    volatile uint32_t& STK_CTRL     = *reinterpret_cast<volatile uint32_t*>(0xE000E010UL);
    volatile uint32_t& STK_LOAD     = *reinterpret_cast<volatile uint32_t*>(0xE000E014UL);
    volatile uint32_t& STK_VAL      = *reinterpret_cast<volatile uint32_t*>(0xE000E018UL);

    // ---- RCC_CR bit definitions ----
    constexpr uint32_t RCC_CR_HSERDY   = (1U << 17U);  ///< HSE oscillator ready flag
    constexpr uint32_t RCC_CR_PLL1RDY  = (1U << 27U);  ///< PLL1 locked flag
    constexpr uint32_t RCC_CR_HSEON    = (1U << 16U);  ///< HSE oscillator enable
    constexpr uint32_t RCC_CR_PLL1ON   = (1U << 24U);  ///< PLL1 enable

    // ---- FLASH_ACR bit definitions (RM0433 §6.10) ----
    constexpr uint32_t FLASH_ACR_LATENCY_MASK = (7U << 0U);   ///< Latency bit-field mask
    constexpr uint32_t FLASH_ACR_WRHIGHFREQ   = (1U << 4U);   ///< Write high-frequency mode
    constexpr uint32_t FLASH_ACR_DCEN         = (1U << 3U);   ///< Data cache enable
    constexpr uint32_t FLASH_ACR_ICEN         = (1U << 1U);   ///< Instruction cache enable

    // ---- PWR_D3CR bit definitions (RM0433 §8.5) ----
    constexpr uint32_t PWR_D3CR_VOS_MASK     = (3U << 0U);    ///< Voltage-scaling bit-field mask
    constexpr uint32_t PWR_D3CR_VOS0         = (0U << 0U);    ///< VOS0 – high performance

    // ---- SysTick control bits (ARM® v7‑M) ----
    constexpr uint32_t STK_CTRL_ENABLE   = (1U << 0U);        ///< Counter enable
    constexpr uint32_t STK_CTRL_TICKINT  = (1U << 1U);        ///< Interrupt on zero
    constexpr uint32_t STK_CTRL_CLKSOURCE = (1U << 2U);       ///< Clock source (1 = CPU clock)

    // ---- PLL divider ratios for 400 MHz from 8 MHz HSE ----
    //     f(VCO) = f(HSE) / M * N = 8 / 4 * 200 = 400 MHz
    //     f(CPU) = f(VCO) / P = 400 / 1 = 400 MHz
    constexpr uint32_t PLL_M = 4U;     ///< PLL1 input division (÷4 → 2 MHz to VCO PFD)
    constexpr uint32_t PLL_N = 200U;   ///< PLL1 VCO multiplication (×200 → 400 MHz VCO)
    constexpr uint32_t PLL_P = 1U;     ///< PLL1 P division (÷1 → 400 MHz core)
    constexpr uint32_t PLL_Q = 2U;     ///< PLL1 Q division (÷2 → 200 MHz)
    constexpr uint32_t PLL_R = 2U;     ///< PLL1 R division (÷2 → 200 MHz)
}

// ============================================================================
//  SystemClock::init   —   Full clock-tree configuration
// ============================================================================

SystemClock::Result SystemClock::init() {
    // Step 1: Enable HSE oscillator, poll until ready
    RCC_CR |= RCC_CR_HSEON;
    if (!waitForHseReady()) {
        return Result::ErrorHse;
    }

    // Step 2: Set voltage regulator to VOS0 (required for ≥ 400 MHz)
    PWR_D3CR = (PWR_D3CR & ~PWR_D3CR_VOS_MASK) | PWR_D3CR_VOS0;
    {
        volatile uint32_t timeout = 1000000;
        while ((PWR_D3CR & PWR_D3CR_VOS_MASK) != PWR_D3CR_VOS0 && --timeout) { }
    }

    // Step 3: Configure Flash for 400 MHz at VOS0: 4 wait-states, caches ON
    configureFlashLatency(0);

    // Step 4: Configure PLL1 and wait for lock
    if (!configurePll()) {
        return Result::ErrorPll;
    }

    // Step 5: Set bus prescalers (RM0433 §5.5.10)
    //   D1 domain  → D1CPRE = /2 (200 MHz), HPRE = /2 (200 MHz)
    //   D2 domain  → APB1DIV = /2 (100 MHz), APB2DIV = /2 (100 MHz)
    //   D3 domain  → APB3DIV = /1 (200 MHz), APB4DIV = /2 (100 MHz)
    RCC_D1CFGR = (1U << 0U) | (8U << 4U);
    RCC_D2CFGR = (4U << 0U) | (4U << 4U);
    RCC_D3CFGR = (0U << 0U) | (4U << 4U);

    // Step 6: Switch system clock source to PLL1 (CFGR.SW = 0b11)
    RCC_CFGR = (RCC_CFGR & ~(3U << 0U)) | (3U << 0U);
    {
        volatile uint32_t timeout = 1000000;
        while (((RCC_CFGR >> 2U) & 3U) != 3U && --timeout) { }
    }

    coreClockHz_ = CPU_FREQ_TARGET;

    // Step 7: Configure SysTick for 1 ms period
    systickCounter_ = 0;
    const uint32_t tickValue = CPU_FREQ_TARGET / 1000U;
    STK_LOAD = tickValue - 1U;
    STK_VAL  = 0U;
    STK_CTRL = STK_CTRL_ENABLE | STK_CTRL_TICKINT | STK_CTRL_CLKSOURCE;

    return Result::Ok;
}

// ============================================================================
//  Time-keeping helpers
// ============================================================================

uint32_t SystemClock::getCoreClockHz() {
    return coreClockHz_;
}

/// Blocking millisecond delay (WFI-based, wakes on any interrupt).
void SystemClock::delayMs(uint32_t ms) {
    uint32_t startTick = systickCounter_;
    while ((systickCounter_ - startTick) < ms) {
        __asm volatile("wfi");
    }
}

/// Blocking microsecond delay (NOP-loop, calibrated for 400 MHz).
void SystemClock::delayUs(uint32_t us) {
    volatile uint32_t cycles = us * 400U;
    while (cycles-- > 0U) {
        __asm volatile("nop");
    }
}

// ============================================================================
//  Private helpers
// ============================================================================

/// Configure Flash ACR: 4 wait-states, D/I-cache, high-frequency write.
void SystemClock::configureFlashLatency(uint32_t voltageScale) {
    (void)voltageScale;
    uint32_t acr = FLASH_ACR & ~FLASH_ACR_LATENCY_MASK;
    acr |= 4U;                               // 4 WS for 400 MHz @ VOS0
    acr |= FLASH_ACR_ICEN | FLASH_ACR_DCEN;  // Enable I-cache & D-cache
    acr |= FLASH_ACR_WRHIGHFREQ;             // High-frequency write mode
    FLASH_ACR = acr;
    volatile uint32_t wait = 100;
    while (--wait) { }
}

/// Wait for HSE ready flag (timeout ≈ 2 million iterations).
bool SystemClock::waitForHseReady() {
    volatile uint32_t timeout = 2000000;
    while (!(RCC_CR & RCC_CR_HSERDY)) {
        if (--timeout == 0) return false;
    }
    return true;
}

/// Configure PLL1 source = HSE, M=4, N=200, P=1, Q=2, R=2, then enable.
bool SystemClock::configurePll() {
    // VCO input = 8 MHz / 4 = 2 MHz PFD
    RCC_PLLCKSELR = (PLL_M << 0U);   // DIVM1 = 4
    RCC_PLLCFGR   = (1U << 0U);      // PLL1SRC = HSE

    // f(VCO) = 2 MHz × 200 = 400 MHz
    // f(P)   = 400 MHz / 1  = 400 MHz (core)
    // f(Q)   = 400 MHz / 2  = 200 MHz
    // f(R)   = 400 MHz / 2  = 200 MHz
    RCC_PLL1DIVR = ((PLL_N - 1U) << 0U) |
                   ((PLL_P - 1U) << 9U) |
                   ((PLL_Q - 1U) << 16U) |
                   ((PLL_R - 1U) << 25U);

    RCC_CR |= RCC_CR_PLL1ON;

    volatile uint32_t timeout = 2000000;
    while (!(RCC_CR & RCC_CR_PLL1RDY)) {
        if (--timeout == 0) return false;
    }
    return true;
}

/// Start LSE oscillator (used by RTC backup domain).
bool SystemClock::configureLse() {
    RCC_BDCR |= (1U << 0U);       // LSEON
    volatile uint32_t timeout = 2000000;
    while (!(RCC_BDCR & (1U << 2U))) {   // LSERDY
        if (--timeout == 0) return false;
    }
    return true;
}

} // namespace drone::drivers

// ============================================================================
//  C-compatible wrappers (for startup-code & interrupt vectors)
// ============================================================================

extern "C" void SystemClock_Config(void) {
    drone::drivers::SystemClock::init();
}

extern "C" void SysTick_Handler(void) {
    ++drone::drivers::SystemClock::systickCounter_;
}
