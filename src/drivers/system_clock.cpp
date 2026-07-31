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
    // Offsets verified against the official STM32H753 CMSIS header
    // (RCC_TypeDef in stm32h753xx.h). CFGR/PLLCKSELR/PLLCFGR/PLL1DIVR/BDCR
    // were previously mis-mapped into the reserved gap between CR and
    // D1CFGR, which meant PLL configuration and the SW/SWS clock-switch
    // never touched real hardware registers.
    volatile uint32_t& RCC_CR       = *reinterpret_cast<volatile uint32_t*>(0x58024400UL);
    volatile uint32_t& RCC_CFGR     = *reinterpret_cast<volatile uint32_t*>(0x58024410UL);
    volatile uint32_t& RCC_PLLCKSELR = *reinterpret_cast<volatile uint32_t*>(0x58024428UL);
    volatile uint32_t& RCC_PLLCFGR  = *reinterpret_cast<volatile uint32_t*>(0x5802442CUL);
    volatile uint32_t& RCC_PLL1DIVR = *reinterpret_cast<volatile uint32_t*>(0x58024430UL);
    volatile uint32_t& RCC_D1CFGR   = *reinterpret_cast<volatile uint32_t*>(0x58024418UL);
    volatile uint32_t& RCC_D2CFGR   = *reinterpret_cast<volatile uint32_t*>(0x5802441CUL);
    volatile uint32_t& RCC_D3CFGR   = *reinterpret_cast<volatile uint32_t*>(0x58024420UL);
    volatile uint32_t& RCC_BDCR     = *reinterpret_cast<volatile uint32_t*>(0x58024470UL);

    // ---- Power controller (RM0433 §8) ----
    // PWR_BASE = 0x58024800; D3CR is at offset 0x18 (CMSIS PWR_TypeDef),
    // not 0x08 (which is CR2/reserved) — the wrong offset meant the VOS
    // write below never touched the real D3CR register.
    volatile uint32_t& PWR_D3CR     = *reinterpret_cast<volatile uint32_t*>(0x58024818UL);

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
    // VOS is bits[15:14] and VOSRDY is bit 13 (CMSIS PWR_D3CR_VOS_Pos=14,
    // PWR_D3CR_VOSRDY_Pos=13) — both were previously at bit 0, so the scale
    // select wrote the wrong bits and the "ready" poll checked a field that
    // never reflects regulator status.
    constexpr uint32_t PWR_D3CR_VOS_MASK     = (3U << 14U);   ///< Voltage-scaling bit-field mask
    constexpr uint32_t PWR_D3CR_VOS0         = (3U << 14U);   ///< VOS0 – high performance (11b)
    constexpr uint32_t PWR_D3CR_VOSRDY       = (1U << 13U);   ///< Voltage regulator output ready

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
    Result result = Result::Ok;
    bool hseReady = false;
    bool pllLocked = false;

    // Step 1: Enable HSE oscillator, poll until ready
    RCC_CR |= RCC_CR_HSEON;
    hseReady = waitForHseReady();
    if (!hseReady) {
        result = Result::ErrorHse;
    }

    if (hseReady) {
        // Step 2: Set voltage regulator to VOS0 (required for ≥ 400 MHz),
        // then wait for VOSRDY — the ready flag, not the selection field,
        // is what actually indicates the regulator has settled.
        PWR_D3CR = (PWR_D3CR & ~PWR_D3CR_VOS_MASK) | PWR_D3CR_VOS0;
        {
            volatile uint32_t timeout = 1000000;
            while (!(PWR_D3CR & PWR_D3CR_VOSRDY) && --timeout) { }
        }

        // Step 3: Configure Flash for 400 MHz at VOS0: 4 wait-states, caches ON
        configureFlashLatency(0);

        // Step 4: Configure PLL1 and wait for lock
        pllLocked = configurePll();
        if (!pllLocked) {
            result = Result::ErrorPll;
        }
    }

    if (hseReady && pllLocked) {
        // Step 5: Set bus prescalers (RM0433 §5.5.10). Field positions and
        // "/2" encodings verified against CMSIS RCC_D1CFGR_HPRE_DIV2 (0x8 at
        // bits[3:0]), RCC_D1CFGR_D1CPRE_DIV2 (0x800, i.e. value 0x4 at
        // bits[10:8]), RCC_D2CFGR_D2PPRE1/2_DIV2 (bits[6:4] / bits[10:8]),
        // and RCC_D3CFGR_D3PPRE_DIV2 (bits[6:4]) — the previous encoding put
        // every field at the wrong bit offset and left D1CPRE at /2 (which
        // would have halved the CPU to 200 MHz, contradicting the 400 MHz
        // CPU_FREQ_TARGET this function reports below).
        //   D1 domain  → D1CPRE = /1 (400 MHz core), HPRE = /2 (200 MHz AHB)
        //   D2 domain  → APB1DIV = /2 (100 MHz), APB2DIV = /2 (100 MHz)
        //   D3 domain  → APB4DIV = /2 (100 MHz)
        RCC_D1CFGR = (8U << 0U) | (0U << 8U);
        RCC_D2CFGR = (4U << 4U) | (4U << 8U);
        RCC_D3CFGR = (4U << 4U);

        // Step 6: Switch system clock source to PLL1 (CFGR.SW = 0b011,
        // bits [2:0]) and wait for CFGR.SWS (bits [5:3]) to confirm the
        // switch. SWS lives 3 bits above SW, not 2 — reading it at the
        // wrong shift meant this loop could never observe a successful
        // switch and always spun to timeout.
        RCC_CFGR = (RCC_CFGR & ~(7U << 0U)) | (3U << 0U);
        {
            volatile uint32_t timeout = 1000000;
            while (((RCC_CFGR >> 3U) & 7U) != 3U && --timeout) { }
        }

        coreClockHz_ = CPU_FREQ_TARGET;
    } else {
        // HSE/PLL bring-up failed: the core is still running on the
        // reset-default HSI oscillator. Record the frequency that is
        // actually active so SysTick below gets a correct reload value
        // instead of assuming the (never reached) 400 MHz PLL clock.
        coreClockHz_ = HSI_VALUE;
    }

    // Step 7: Configure SysTick for 1 ms period. This always runs — even on
    // a clock-configuration failure — because every delayMs()/getTickMs()
    // caller (including the flight-controller failsafe timeout) depends on
    // it; leaving SysTick disabled would hang the whole system permanently
    // instead of just running at a lower, safe clock rate.
    systickCounter_ = 0;
    const uint32_t tickValue = coreClockHz_ / 1000U;
    STK_LOAD = tickValue - 1U;
    STK_VAL  = 0U;
    STK_CTRL = STK_CTRL_ENABLE | STK_CTRL_TICKINT | STK_CTRL_CLKSOURCE;

    return result;
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
    // PLLCKSELR: PLLSRC is bits[1:0] (2 = HSE), DIVM1 is bits[9:4] — both
    // were previously written into the wrong bit positions (PLLSRC ended
    // up in PLLCFGR, DIVM1 in PLLCKSELR bits[3:0]), so the PLL never
    // actually read from HSE with the intended /4 pre-divider.
    // VCO input = 8 MHz / 4 = 2 MHz PFD
    RCC_PLLCKSELR = (2U << 0U) | (PLL_M << 4U);   // PLLSRC = HSE, DIVM1 = 4

    // DIVP1EN/DIVQ1EN/DIVR1EN (bits 16/17/18) must be set or the PLL
    // produces no P/Q/R output at all — SYSCLK would never actually
    // receive a clock after switching SW to PLL1.
    RCC_PLLCFGR = (1U << 16U) | (1U << 17U) | (1U << 18U);

    // f(VCO) = 2 MHz × 200 = 400 MHz
    // f(P)   = 400 MHz / 1  = 400 MHz (core)
    // f(Q)   = 400 MHz / 2  = 200 MHz
    // f(R)   = 400 MHz / 2  = 200 MHz
    // R1 field is bits[31:24], not [31:25] — the extra shift clipped R's
    // top bit into the reserved bit above the field.
    RCC_PLL1DIVR = ((PLL_N - 1U) << 0U) |
                   ((PLL_P - 1U) << 9U) |
                   ((PLL_Q - 1U) << 16U) |
                   ((PLL_R - 1U) << 24U);

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
    while (!(RCC_BDCR & (1U << 1U))) {   // LSERDY (bit 1, not bit 2)
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
