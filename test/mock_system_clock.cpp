// Mock SystemClock for unit testing
// Provides stub implementations of hardware-dependent functions

#include "drivers/system_clock.hpp"
#include <cstdint>

namespace drone::drivers {

// Define the static members
volatile uint32_t SystemClock::systickCounter_ = 0;
uint32_t SystemClock::coreClockHz_ = 400000000UL;

// Mock implementations
SystemClock::Result SystemClock::init() {
    return SystemClock::Result::Ok;
}

uint32_t SystemClock::getCoreClockHz() {
    return coreClockHz_;
}

void SystemClock::delayMs(uint32_t ms) {
    systickCounter_ += ms;
}

void SystemClock::delayUs(uint32_t us) {
    systickCounter_ += us / 1000;
}

void SystemClock::configureFlashLatency(uint32_t) {
    // No-op in test
}

bool SystemClock::waitForHseReady() {
    return true;
}

bool SystemClock::configurePll() {
    return true;
}

bool SystemClock::configureLse() {
    return true;
}

} // namespace drone::drivers

// Legacy C-compatible function for compatibility
extern "C" void SystemClock_Config(void) {
    drone::drivers::SystemClock::init();
}

extern "C" void SysTick_Handler(void) {
    // No-op in test
}
