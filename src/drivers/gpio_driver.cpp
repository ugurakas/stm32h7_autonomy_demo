#include "drivers/gpio_driver.hpp"

#include <cstdint>

namespace drone::drivers {

namespace {
    constexpr uint32_t GPIO_MODER_OFF = 0x00U;
    constexpr uint32_t GPIO_OTYPER_OFF = 0x04U;
    constexpr uint32_t GPIO_OSPEEDR_OFF = 0x08U;
    constexpr uint32_t GPIO_PUPDR_OFF = 0x0CU;
    constexpr uint32_t GPIO_IDR_OFF = 0x10U;
    constexpr uint32_t GPIO_ODR_OFF = 0x14U;
    constexpr uint32_t GPIO_BSRR_OFF = 0x18U;
    constexpr uint32_t GPIO_AFRL_OFF = 0x20U;
    constexpr uint32_t GPIO_AFRH_OFF = 0x24U;
    
    volatile uint32_t& RCC_AHB4ENR = *reinterpret_cast<volatile uint32_t*>(0x580244E0UL);
}

GpioDriver::GpioDriver(GpioPort port, uint32_t pin)
    : port_(port), pin_(pin) {
    enableClock();
}

void GpioDriver::configure(const GpioConfig& config) {
    volatile uint32_t* base = getBaseAddr();
    if (!base) return;
    
    // Configure mode
    uint32_t modeShift = pin_ * 2;
    base[GPIO_MODER_OFF / 4U] &= ~(3U << modeShift);
    base[GPIO_MODER_OFF / 4U] |= (config.mode << modeShift);
    
    // Configure output type
    base[GPIO_OTYPER_OFF / 4U] &= ~(1U << pin_);
    base[GPIO_OTYPER_OFF / 4U] |= (config.outputType << pin_);
    
    // Configure speed
    uint32_t speedShift = pin_ * 2;
    base[GPIO_OSPEEDR_OFF / 4U] &= ~(3U << speedShift);
    base[GPIO_OSPEEDR_OFF / 4U] |= (config.speed << speedShift);
    
    // Configure pull-up/down
    uint32_t pullShift = pin_ * 2;
    base[GPIO_PUPDR_OFF / 4U] &= ~(3U << pullShift);
    base[GPIO_PUPDR_OFF / 4U] |= (config.pull << pullShift);
    
    // Configure alternate function
    if (config.mode == 2) { // Alternate function mode
        if (pin_ <= 7) {
            uint32_t afShift = pin_ * 4;
            base[GPIO_AFRL_OFF / 4U] &= ~(0xFU << afShift);
            base[GPIO_AFRL_OFF / 4U] |= (config.alternate << afShift);
        } else {
            uint32_t afShift = (pin_ - 8) * 4;
            base[GPIO_AFRH_OFF / 4U] &= ~(0xFU << afShift);
            base[GPIO_AFRH_OFF / 4U] |= (config.alternate << afShift);
        }
    }
}

void GpioDriver::set() {
    volatile uint32_t* base = getBaseAddr();
    if (!base) return;
    base[GPIO_BSRR_OFF / 4U] = (1U << pin_);
}

void GpioDriver::reset() {
    volatile uint32_t* base = getBaseAddr();
    if (!base) return;
    base[GPIO_BSRR_OFF / 4U] = (1U << (pin_ + 16));
}

void GpioDriver::toggle() {
    volatile uint32_t* base = getBaseAddr();
    if (!base) return;
    if (base[GPIO_ODR_OFF / 4U] & (1U << pin_)) {
        reset();
    } else {
        set();
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

void GpioDriver::writePin(GpioPort port, uint32_t pin, bool value) {
    GpioDriver gpio(port, pin);
    gpio.write(value);
}

bool GpioDriver::readPin(GpioPort port, uint32_t pin) {
    GpioDriver gpio(port, pin);
    return gpio.read();
}

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
    uint32_t bit = (1U << static_cast<uint32_t>(port_));
    RCC_AHB4ENR |= bit;
}

} // namespace drone::drivers
