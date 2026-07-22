#include "drivers/system_clock.hpp"

#include <cstdint>

namespace drone::drivers {

volatile uint32_t SystemClock::systickCounter_ = 0;
uint32_t SystemClock::coreClockHz_ = 0;

// STM32H7 registers (simplified for template - real implementation would use CMSIS/HAL)
namespace {
    // Flash register base
    volatile uint32_t& FLASH_ACR = *reinterpret_cast<volatile uint32_t*>(0x52002000UL);
    
    // Reset and Clock Control (RCC) registers
    volatile uint32_t& RCC_CR = *reinterpret_cast<volatile uint32_t*>(0x58024400UL);
    volatile uint32_t& RCC_CFGR = *reinterpret_cast<volatile uint32_t*>(0x58024408UL);
    volatile uint32_t& RCC_PLLCKSELR = *reinterpret_cast<volatile uint32_t*>(0x5802440CUL);
    volatile uint32_t& RCC_PLLCFGR = *reinterpret_cast<volatile uint32_t*>(0x58024410UL);
    volatile uint32_t& RCC_PLL1DIVR = *reinterpret_cast<volatile uint32_t*>(0x58024414UL);
    volatile uint32_t& RCC_D1CFGR = *reinterpret_cast<volatile uint32_t*>(0x58024418UL);
    volatile uint32_t& RCC_D2CFGR = *reinterpret_cast<volatile uint32_t*>(0x5802441CUL);
    volatile uint32_t& RCC_D3CFGR = *reinterpret_cast<volatile uint32_t*>(0x58024420UL);
    volatile uint32_t& RCC_BDCR = *reinterpret_cast<volatile uint32_t*>(0x580244A0UL);
    volatile uint32_t& PWR_D3CR = *reinterpret_cast<volatile uint32_t*>(0x58024808UL);
    
    // System Control Block
    volatile uint32_t& STK_CTRL = *reinterpret_cast<volatile uint32_t*>(0xE000E010UL);
    volatile uint32_t& STK_LOAD = *reinterpret_cast<volatile uint32_t*>(0xE000E014UL);
    volatile uint32_t& STK_VAL = *reinterpret_cast<volatile uint32_t*>(0xE000E018UL);
    
    // Register bit definitions
    constexpr uint32_t RCC_CR_HSERDY = (1U << 17U);
    constexpr uint32_t RCC_CR_PLL1RDY = (1U << 27U);
    constexpr uint32_t RCC_CR_HSEON = (1U << 16U);
    constexpr uint32_t RCC_CR_PLL1ON = (1U << 24U);
    
    constexpr uint32_t FLASH_ACR_LATENCY_MASK = (7U << 0U);
    constexpr uint32_t FLASH_ACR_WRHIGHFREQ = (1U << 4U);
    constexpr uint32_t FLASH_ACR_DCEN = (1U << 3U);
    constexpr uint32_t FLASH_ACR_ICEN = (1U << 1U);
    
    constexpr uint32_t PWR_D3CR_VOS_MASK = (3U << 0U);
    constexpr uint32_t PWR_D3CR_VOS0 = (0U << 0U);
    
    // SysTick control bits
    constexpr uint32_t STK_CTRL_ENABLE = (1U << 0U);
    constexpr uint32_t STK_CTRL_TICKINT = (1U << 1U);
    constexpr uint32_t STK_CTRL_CLKSOURCE = (1U << 2U);
    
    // PLL configuration for 400 MHz from 8 MHz HSE
    constexpr uint32_t PLL_M = 4U;
    constexpr uint32_t PLL_N = 200U;
    constexpr uint32_t PLL_P = 1U;
    constexpr uint32_t PLL_Q = 2U;
    constexpr uint32_t PLL_R = 2U;
}

SystemClock::Result SystemClock::init() {
    // 1. Enable HSE oscillator and wait for ready
    RCC_CR |= RCC_CR_HSEON;
    if (!waitForHseReady()) {
        return Result::ErrorHse;
    }
    
    // 2. Configure voltage scaling for high frequency (VOS0)
    PWR_D3CR = (PWR_D3CR & ~PWR_D3CR_VOS_MASK) | PWR_D3CR_VOS0;
    volatile uint32_t timeout = 1000000;
    while ((PWR_D3CR & PWR_D3CR_VOS_MASK) != PWR_D3CR_VOS0 && --timeout);
    
    // 3. Configure Flash latency for 400 MHz at VOS0 (4 wait states)
    configureFlashLatency(0);
    
    // 4. Configure and enable PLL1
    if (!configurePll()) {
        return Result::ErrorPll;
    }
    
    // 5. Configure bus prescalers
    // D1 domain: CPU/D1 buses at 200 MHz, AHB at 200 MHz
    // D2 domain: APB1 at 100 MHz, APB2 at 100 MHz
    // D3 domain: APB3 at 200 MHz, APB4 at 100 MHz
    RCC_D1CFGR = (1U << 0U) | (8U << 4U);   // D1CPRE=/2, HPRE=/2
    RCC_D2CFGR = (4U << 0U) | (4U << 4U);   // APB1=/2, APB2=/2
    RCC_D3CFGR = (0U << 0U) | (4U << 4U);   // APB3=/1, APB4=/2
    
    // 6. Select PLL1 as system clock (SW = 0b11)
    RCC_CFGR = (RCC_CFGR & ~(3U << 0U)) | (3U << 0U);
    
    timeout = 1000000;
    while (((RCC_CFGR >> 2U) & 3U) != 3U && --timeout);
    
    coreClockHz_ = CPU_FREQ_TARGET;
    
    // 7. Configure SysTick for 1ms interrupts
    systickCounter_ = 0;
    const uint32_t tickValue = CPU_FREQ_TARGET / 1000U;
    STK_LOAD = tickValue - 1U;
    STK_VAL = 0U;
    STK_CTRL = STK_CTRL_ENABLE | STK_CTRL_TICKINT | STK_CTRL_CLKSOURCE;
    
    return Result::Ok;
}

uint32_t SystemClock::getCoreClockHz() {
    return coreClockHz_;
}

void SystemClock::delayMs(uint32_t ms) {
    uint32_t startTick = systickCounter_;
    while ((systickCounter_ - startTick) < ms) {
        __asm volatile("wfi");
    }
}

void SystemClock::delayUs(uint32_t us) {
    volatile uint32_t cycles = us * 400U;
    while (cycles-- > 0U) {
        __asm volatile("nop");
    }
}

void SystemClock::configureFlashLatency(uint32_t voltageScale) {
    (void)voltageScale;
    uint32_t acr = FLASH_ACR & ~FLASH_ACR_LATENCY_MASK;
    acr |= 4U;  // 4 wait states for 400MHz @ VOS0
    acr |= FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    acr |= FLASH_ACR_WRHIGHFREQ;
    FLASH_ACR = acr;
    
    volatile uint32_t wait = 100;
    while (--wait);
}

bool SystemClock::waitForHseReady() {
    volatile uint32_t timeout = 2000000;
    while (!(RCC_CR & RCC_CR_HSERDY)) {
        if (--timeout == 0) return false;
    }
    return true;
}

bool SystemClock::configurePll() {
    // Configure PLL source and dividers for 8MHz HSE -> 400MHz core
    RCC_PLLCKSELR = (PLL_M << 0U);  // DIVM1 = 4
    RCC_PLLCFGR = (1U << 0U);       // PLL1SRC = HSE
    
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

bool SystemClock::configureLse() {
    RCC_BDCR |= (1U << 0U);  // LSEON
    volatile uint32_t timeout = 2000000;
    while (!(RCC_BDCR & (1U << 2U))) {
        if (--timeout == 0) return false;
    }
    return true;
}

} // namespace drone::drivers

// Legacy C-compatible functions
extern "C" void SystemClock_Config(void) {
    drone::drivers::SystemClock::init();
}

extern "C" void SysTick_Handler(void) {
    ++drone::drivers::SystemClock::systickCounter_;
}
