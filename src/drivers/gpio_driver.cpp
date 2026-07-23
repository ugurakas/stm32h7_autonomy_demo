/**
 * @file    gpio_driver.cpp
 * @brief   Implementation of the STM32H7 GPIO driver.
 *
 * @details Implements register-level GPIO operations:
 *          - 11 GPIO ports (A–K) with clock enable via RCC_AHB4ENR
 *          - MODER, OTYPER, OSPEEDR, PUPDR, AFR register access
 *          - Atomic set/reset via BSRR (set upper/reset lower half)
 *          - Toggle via ODR
 *          - Static helpers for fast one-shot pin operations
 *
 *          Register addresses conform to RM0433 §10 (GPIO).
 *
 * @ingroup drivers
 */

#include "drivers/gpio_driver.hpp"

#include <cstdint>

namespace drone::drivers {

// ---- GPIO register offsets (RM0433 §10.4) ----
namespace {
    constexpr uint32_t GPIO_MODER_OFF   = 0x00U;   ///< Mode register
    constexpr uint32_t GPIO_OTYPER_OFF  = 0x04U;   ///< Output type register
    constexpr uint32_t GPIO_OSPEEDR_OFF = 0x08U;   ///< Output speed register
    constexpr uint32_t GPIO_PUPDR_OFF   = 0x0CU;   ///< Pull-up/down register
    constexpr uint32_t GPIO_IDR_OFF     = 0x10U;   ///< Input data register
    constexpr uint32_t GPIO_ODR_OFF     = 0x14U;   ///< Output data register
    constexpr uint32_t GPIO_BSRR_OFF    = 0x18U;   ///< Bit set/reset register
    constexpr uint32_t GPIO_AFRL_OFF    = 0x20U;   ///< Alternate function low (pins 0–7)
    constexpr uint32_t GPIO_AFRH_OFF    = 0x24U;   ///< Alternate function high (pins 8–15)

    // RCC AHB4 clock enable register (RM0433 §5.8)
    volatile uint32_t& RCC_AHB4ENR = *reinterpret_cast<volatile uint32_t*>(0x580244E0UL);
}

GpioDriver::GpioDriver(GpioPort port, uint32_t pin)
    : port_(port), pin_(pin) {
}

void GpioDriver::configure(const GpioConfig& config) {
    volatile uint32_t* base = getBaseAddr();
    if (!base) return;

    enableClock();

    // MODER: 2 bits per pin
    uint32_t moder = base[GPIO_MODER_OFF / 4U];
    moder &= ~(3U << (pin_ * 2U));
    moder |= (config.mode & 3U) << (pin_ * 2U);
    base[GPIO_MODER_OFF / 4U] = moder;

    // OTYPER: 1 bit per pin
    if (config.mode == 1) {  // Output
        uint32_t otyper = base[GPIO_OTYPER_OFF / 4U];
        otyper &= ~(1U << pin_);
        otyper |= (config.outputType & 1U) << pin_;
        base[GPIO_OTYPER_OFF / 4U] = otyper;
    }

    // OSPEEDR: 2 bits per pin
    uint32_t ospeedr = base[GPIO_OSPEEDR_OFF / 4U];
    ospeedr &= ~(3U << (pin_ * 2U));
    ospeedr |= (config.speed & 3U) << (pin_ * 2U);
    base[GPIO_OSPEEDR_OFF / 4U] = ospeedr;

    // PUPDR: 2 bits per pin
    uint32_t pupdr = base[GPIO_PUPDR_OFF / 4U];
    pupdr &= ~(3U << (pin_ * 2U));
    pupdr |= (config.pull & 3U) << (pin_ * 2U);
    base[GPIO_PUPDR_OFF / 4U] = pupdr;

    // AFR: 4 bits per pin
    if (config.mode == 2) {  // Alternate function
        if (pin_ <= 7) {
            uint32_t afrl = base[GPIO_AFRL_OFF / 4U];
            afrl &= ~(0xFU << (pin_ * 4U));
            afrl |= (config.alternate & 0xFU) << (pin_ * 4U);
            base[GPIO_AFRL_OFF / 4U] = afrl;
        } else {
            uint32_t afrh = base[GPIO_AFRH_OFF / 4U];
            afrh &= ~(0xFU << ((pin_ - 8U) * 4U));
            afrh |= (config.alternate & 0xFU) << ((pin_ - 8U) * 4U);
            base[GPIO_AFRH_OFF / 4U] = afrh;
        }
    }
}

void GpioDriver::set() {
    volatile uint32_t* base = getBaseAddr();
    if (base) {
        base[GPIO_BSRR_OFF / 4U] = (1U << pin_);         // BSRR low half = set
    }
}

void GpioDriver::reset() {
    volatile uint32_t* base = getBaseAddr();
    if (base) {
        base[GPIO_BSRR_OFF / 4U] = (1U << (pin_ + 16U));  // BSRR high half = reset
    }
}

void GpioDriver::toggle() {
    volatile uint32_t* base = getBaseAddr();
    if (base) {
        base[GPIO_ODR_OFF / 4U] ^= (1U << pin_);
    }
}

void GpioDriver::write(bool value) {
    if (value) {
        set();
    } else {
        reset();
    }
}

bool GpioDriver::read() const {
    volatile uint32_t* base = getBaseAddr();
    if (!base) return false;
    return (base[GPIO_IDR_OFF / 4U] & (1U << pin_)) != 0;
}

// ---------------------------------------------------------------------------
//  Static helpers
// ---------------------------------------------------------------------------

void GpioDriver::writePin(GpioPort port, uint32_t pin, bool value) {
    GpioDriver gpio(port, pin);
    gpio.write(value);
}

bool GpioDriver::readPin(GpioPort port, uint32_t pin) {
    GpioDriver gpio(port, pin);
    return gpio.read();
}

// ---------------------------------------------------------------------------
//  Register access helpers
// ---------------------------------------------------------------------------

volatile uint32_t* GpioDriver::getBaseAddr() const {
    switch (port_) {
        case GpioPort::PortA: return reinterpret_cast<volatile uint32_t*>(GPIOA_BASE);
        case GpioPort::PortB: return reinterpret_cast<volatile uint32_t*>(GPIOB_BASE);
        case GpioPort::PortC: return reinterpret_cast<volatile uint32_t*>(GPIOC_BASE);
        case GpioPort::PortD: return reinterpret_cast<volatile uint32_t*>(GPIOD_BASE);
        case GpioPort::PortE: return reinterpret_cast<volatile uint32_t*>(GPIOE_BASE);
        case GpioPort::PortF: return reinterpret_cast<volatile uint32_t*>(GPIOF_BASE);
        case GpioPort::PortG: return reinterpret_cast<volatile uint32_t*>(GPIOG_BASE);
        case GpioPort::PortH: return reinterpret_cast<volatile uint32_t*>(GPIOH_BASE);
        case GpioPort::PortI: return reinterpret_cast<volatile uint32_t*>(GPIOI_BASE);
        case GpioPort::PortJ: return reinterpret_cast<volatile uint32_t*>(GPIOJ_BASE);
        case GpioPort::PortK: return reinterpret_cast<volatile uint32_t*>(GPIOK_BASE);
        default: return nullptr;
    }
}

void GpioDriver::enableClock() {
    // AHB4ENR bits: port A=0, B=1, ..., K=10
    RCC_AHB4ENR |= (1U << static_cast<uint32_t>(port_));
}

} // namespace drone::drivers
