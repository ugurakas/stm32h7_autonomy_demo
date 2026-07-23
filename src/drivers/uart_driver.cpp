#include "drivers/uart_driver.hpp"

#include <cstdint>
#include <cstring>

namespace drone::drivers {

namespace {
    constexpr uint32_t USART1_BASE = 0x40011000UL;
    constexpr uint32_t USART2_BASE = 0x40004400UL;
    constexpr uint32_t USART3_BASE = 0x40004800UL;
    constexpr uint32_t UART4_BASE = 0x40004C00UL;
    constexpr uint32_t UART5_BASE = 0x40005000UL;
    constexpr uint32_t USART6_BASE = 0x40011400UL;
    constexpr uint32_t UART7_BASE = 0x40007800UL;
    constexpr uint32_t UART8_BASE = 0x40007C00UL;
    
    constexpr uint32_t USART_CR1_OFF = 0x00U;
    constexpr uint32_t USART_CR2_OFF = 0x04U;
    constexpr uint32_t USART_CR3_OFF = 0x08U;
    constexpr uint32_t USART_BRR_OFF = 0x0CU;
    constexpr uint32_t USART_ISR_OFF = 0x1CU;
    constexpr uint32_t USART_ICR_OFF = 0x20U;
    constexpr uint32_t USART_RDR_OFF = 0x24U;
    constexpr uint32_t USART_TDR_OFF = 0x28U;
    
    constexpr uint32_t USART_CR1_UE = (1U << 0U);
    constexpr uint32_t USART_CR1_RE = (1U << 2U);
    constexpr uint32_t USART_CR1_TE = (1U << 3U);
    constexpr uint32_t USART_CR1_RXNEIE = (1U << 5U);
    constexpr uint32_t USART_CR1_TCIE = (1U << 6U);
    constexpr uint32_t USART_CR1_PEIE = (1U << 8U);
    constexpr uint32_t USART_CR1_M0 = (1U << 12U);
    constexpr uint32_t USART_CR1_M1 = (1U << 28U);
    
    constexpr uint32_t USART_CR2_STOP_2 = (2U << 12U);
    
    constexpr uint32_t USART_CR3_DMAR = (1U << 6U);
    constexpr uint32_t USART_CR3_DMAT = (1U << 7U);
    
    constexpr uint32_t USART_ISR_TXE = (1U << 7U);
    constexpr uint32_t USART_ISR_TC = (1U << 6U);
    constexpr uint32_t USART_ISR_RXNE = (1U << 5U);
    constexpr uint32_t USART_ISR_ORE = (1U << 3U);
    constexpr uint32_t USART_ISR_FE = (1U << 1U);
    constexpr uint32_t USART_ISR_PE = (1U << 0U);
    
    constexpr uint32_t USART_ICR_TCCF = (1U << 6U);
    constexpr uint32_t USART_ICR_ORECF = (1U << 3U);
    constexpr uint32_t USART_ICR_FECF = (1U << 1U);
    constexpr uint32_t USART_ICR_PECF = (1U << 0U);
    
    constexpr uint32_t RCC_APB2ENR_USART1 = (1U << 4U);
    constexpr uint32_t RCC_APB2ENR_USART6 = (1U << 5U);
    constexpr uint32_t RCC_APB1LENR_USART2 = (1U << 17U);
    constexpr uint32_t RCC_APB1LENR_USART3 = (1U << 18U);
    constexpr uint32_t RCC_APB1LENR_UART4 = (1U << 19U);
    constexpr uint32_t RCC_APB1LENR_UART5 = (1U << 20U);
    constexpr uint32_t RCC_APB1LENR_UART7 = (1U << 30U);
    constexpr uint32_t RCC_APB1LENR_UART8 = (1U << 31U);
    
    // STM32H7 RCC register map (RM0433):
    // RCC_APB1LENR1 = 0x58024464, RCC_APB2ENR = 0x580244A0, RCC_AHB4ENR = 0x580244E0
    volatile uint32_t& RCC_APB1LENR = *reinterpret_cast<volatile uint32_t*>(0x58024464UL);
    volatile uint32_t& RCC_APB2ENR = *reinterpret_cast<volatile uint32_t*>(0x580244A0UL);
    volatile uint32_t& RCC_AHB4ENR = *reinterpret_cast<volatile uint32_t*>(0x580244E0UL);
}

UartDriver::UartDriver(uint32_t instance)
    : instance_(instance), initialized_(false), config_(),
      rxBuffer_(nullptr), rxHead_(0), rxTail_(0), rxBufferSize_(0) {
}

UartDriver::~UartDriver() {
    if (initialized_) {
        deinit();
    }
    delete[] rxBuffer_;
}

UartDriver::Result UartDriver::init(const UartConfig& config) {
    if (initialized_) {
        return Result::ErrorBusy;
    }
    
    config_ = config;
    rxBufferSize_ = config_.rxBufferSize;
    
    delete[] rxBuffer_;
    rxBuffer_ = new uint8_t[rxBufferSize_];
    rxHead_ = 0;
    rxTail_ = 0;
    
    enableClock();
    configureGpio();
    
    volatile uint32_t* base = getBaseAddr();
    if (!base) {
        return Result::ErrorInvalidParam;
    }
    
    base[USART_CR1_OFF / 4U] = 0;
    
    uint32_t brr = 100000000UL / config_.baudrate;
    base[USART_BRR_OFF / 4U] = brr;
    
    uint32_t cr1 = USART_CR1_RE | USART_CR1_TE;
    if (config_.wordLength == 9) {
        cr1 |= USART_CR1_M0;
    } else if (config_.wordLength == 7) {
        cr1 |= USART_CR1_M1;
    }
    base[USART_CR1_OFF / 4U] = cr1;
    
    uint32_t cr2 = 0;
    if (config_.stopBits == 2) {
        cr2 |= USART_CR2_STOP_2;
    }
    base[USART_CR2_OFF / 4U] = cr2;
    
    uint32_t cr3 = 0;
    if (config_.useRxDma) {
        cr3 |= USART_CR3_DMAR;
    }
    if (config_.useTxDma) {
        cr3 |= USART_CR3_DMAT;
    }
    base[USART_CR3_OFF / 4U] = cr3;
    
    base[USART_ICR_OFF / 4U] = 0xFFFFFFFFU;
    base[USART_CR1_OFF / 4U] |= USART_CR1_RXNEIE;
    base[USART_CR1_OFF / 4U] |= USART_CR1_UE;
    
    initialized_ = true;
    return Result::Ok;
}

UartDriver::Result UartDriver::deinit() {
    if (!initialized_) {
        return Result::Ok;
    }
    volatile uint32_t* base = getBaseAddr();
    if (base) {
        base[USART_CR1_OFF / 4U] = 0;
    }
    initialized_ = false;
    return Result::Ok;
}

UartDriver::Result UartDriver::transmit(const uint8_t* data, uint16_t size, uint32_t timeoutMs) {
    volatile uint32_t* base = getBaseAddr();
    if (!base || !data) return Result::ErrorInvalidParam;
    
    for (uint16_t i = 0; i < size; ++i) {
        uint32_t timeout = timeoutMs * 1000;
        while (!(base[USART_ISR_OFF / 4U] & USART_ISR_TXE)) {
            if (base[USART_ISR_OFF / 4U] & USART_ISR_ORE) {
                base[USART_ICR_OFF / 4U] = USART_ICR_ORECF;
                return Result::ErrorOverrun;
            }
            if (--timeout == 0) return Result::ErrorTimeout;
        }
        base[USART_TDR_OFF / 4U] = data[i];
    }
    
    uint32_t timeout = timeoutMs * 1000;
    while (!(base[USART_ISR_OFF / 4U] & USART_ISR_TC)) {
        if (--timeout == 0) return Result::ErrorTimeout;
    }
    return Result::Ok;
}

UartDriver::Result UartDriver::receive(uint8_t* data, uint16_t size, uint32_t timeoutMs) {
    volatile uint32_t* base = getBaseAddr();
    if (!base || !data) return Result::ErrorInvalidParam;
    
    for (uint16_t i = 0; i < size; ++i) {
        uint32_t timeout = timeoutMs * 1000;
        while (!(base[USART_ISR_OFF / 4U] & USART_ISR_RXNE)) {
            if (base[USART_ISR_OFF / 4U] & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_PE)) {
                base[USART_ICR_OFF / 4U] = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_PECF;
                if (base[USART_ISR_OFF / 4U] & USART_ISR_ORE) return Result::ErrorOverrun;
                if (base[USART_ISR_OFF / 4U] & USART_ISR_FE) return Result::ErrorFraming;
                if (base[USART_ISR_OFF / 4U] & USART_ISR_PE) return Result::ErrorParity;
            }
            if (--timeout == 0) return Result::ErrorTimeout;
        }
        data[i] = (uint8_t)(base[USART_RDR_OFF / 4U] & 0xFFU);
    }
    return Result::Ok;
}

uint16_t UartDriver::getRxAvailable() const {
    if (rxHead_ >= rxTail_) {
        return rxHead_ - rxTail_;
    }
    return rxBufferSize_ - (rxTail_ - rxHead_);
}

uint16_t UartDriver::readRxBuffer(uint8_t* data, uint16_t maxSize) {
    uint16_t available = getRxAvailable();
    uint16_t readSize = (available < maxSize) ? available : maxSize;
    for (uint16_t i = 0; i < readSize; ++i) {
        data[i] = rxBuffer_[rxTail_];
        rxTail_ = (rxTail_ + 1) % rxBufferSize_;
    }
    return readSize;
}

void UartDriver::enableInterrupts(bool enable) {
    volatile uint32_t* base = getBaseAddr();
    if (!base) return;
    if (enable) {
        base[USART_CR1_OFF / 4U] |= USART_CR1_RXNEIE | USART_CR1_TCIE | USART_CR1_PEIE;
    } else {
        base[USART_CR1_OFF / 4U] &= ~(USART_CR1_RXNEIE | USART_CR1_TCIE | USART_CR1_PEIE);
    }
}

void UartDriver::onDataReceived(uint8_t byte) {
    uint16_t nextHead = (rxHead_ + 1) % rxBufferSize_;
    if (nextHead != rxTail_) {
        rxBuffer_[rxHead_] = byte;
        rxHead_ = nextHead;
    }
}

volatile uint32_t* UartDriver::getBaseAddr() const {
    switch (instance_) {
        case 1: return reinterpret_cast<volatile uint32_t*>(USART1_BASE);
        case 2: return reinterpret_cast<volatile uint32_t*>(USART2_BASE);
        case 3: return reinterpret_cast<volatile uint32_t*>(USART3_BASE);
        case 4: return reinterpret_cast<volatile uint32_t*>(UART4_BASE);
        case 5: return reinterpret_cast<volatile uint32_t*>(UART5_BASE);
        case 6: return reinterpret_cast<volatile uint32_t*>(USART6_BASE);
        case 7: return reinterpret_cast<volatile uint32_t*>(UART7_BASE);
        case 8: return reinterpret_cast<volatile uint32_t*>(UART8_BASE);
        default: return nullptr;
    }
}

void UartDriver::enableClock() {
    switch (instance_) {
        case 1: RCC_APB2ENR |= RCC_APB2ENR_USART1; break;
        case 2: RCC_APB1LENR |= RCC_APB1LENR_USART2; break;
        case 3: RCC_APB1LENR |= RCC_APB1LENR_USART3; break;
        case 4: RCC_APB1LENR |= RCC_APB1LENR_UART4; break;
        case 5: RCC_APB1LENR |= RCC_APB1LENR_UART5; break;
        case 6: RCC_APB2ENR |= RCC_APB2ENR_USART6; break;
        case 7: RCC_APB1LENR |= RCC_APB1LENR_UART7; break;
        case 8: RCC_APB1LENR |= RCC_APB1LENR_UART8; break;
    }
}

void UartDriver::configureGpio() {
    RCC_AHB4ENR |= (1U << 0U) | (1U << 1U) | (1U << 2U) | (1U << 3U) | (1U << 4U);
}

uint32_t UartDriver::computeBaudDiv(uint32_t baudrate) const {
    return 100000000UL / baudrate;
}

} // namespace drone::drivers
