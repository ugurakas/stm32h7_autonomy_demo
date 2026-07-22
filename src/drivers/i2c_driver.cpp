#include "drivers/i2c_driver.hpp"

#include <cstdint>

namespace drone::drivers {

namespace {
    // I2C register offsets
    constexpr uint32_t I2C_CR1_OFF = 0x00U;
    constexpr uint32_t I2C_CR2_OFF = 0x04U;
    constexpr uint32_t I2C_OAR1_OFF = 0x08U;
    constexpr uint32_t I2C_TIMINGR_OFF = 0x10U;
    constexpr uint32_t I2C_ISR_OFF = 0x18U;
    constexpr uint32_t I2C_ICR_OFF = 0x1CU;
    constexpr uint32_t I2C_TXDR_OFF = 0x28U;
    constexpr uint32_t I2C_RXDR_OFF = 0x2CU;
    
    // CR1 bits
    constexpr uint32_t I2C_CR1_PE = (1U << 0U);
    constexpr uint32_t I2C_CR1_TXIE = (1U << 1U);
    constexpr uint32_t I2C_CR1_RXIE = (1U << 2U);
    constexpr uint32_t I2C_CR1_NACKIE = (1U << 4U);
    constexpr uint32_t I2C_CR1_STOPIE = (1U << 5U);
    constexpr uint32_t I2C_CR1_TCIE = (1U << 6U);
    constexpr uint32_t I2C_CR1_ERRIE = (1U << 7U);
    constexpr uint32_t I2C_CR1_ANFOFF = (1U << 12U);
    constexpr uint32_t I2C_CR1_SWRST = (1U << 15U);
    
    // CR2 bits
    constexpr uint32_t I2C_CR2_SADD_POS = 0U;
    constexpr uint32_t I2C_CR2_SADD_MASK = (0x3FFU << 0U);
    constexpr uint32_t I2C_CR2_RD_WRN = (1U << 10U);
    constexpr uint32_t I2C_CR2_ADD10 = (1U << 11U);
    constexpr uint32_t I2C_CR2_HEAD10R = (1U << 12U);
    constexpr uint32_t I2C_CR2_START = (1U << 13U);
    constexpr uint32_t I2C_CR2_STOP = (1U << 14U);
    constexpr uint32_t I2C_CR2_NACK = (1U << 15U);
    constexpr uint32_t I2C_CR2_NBYTES_POS = 16U;
    constexpr uint32_t I2C_CR2_NBYTES_MASK = (0xFFU << 16U);
    constexpr uint32_t I2C_CR2_RELOAD = (1U << 24U);
    constexpr uint32_t I2C_CR2_AUTOEND = (1U << 25U);
    constexpr uint32_t I2C_CR2_PECBYTE = (1U << 26U);
    
    // ISR bits
    constexpr uint32_t I2C_ISR_TXE = (1U << 0U);
    constexpr uint32_t I2C_ISR_TXIS = (1U << 1U);
    constexpr uint32_t I2C_ISR_RXNE = (1U << 2U);
    constexpr uint32_t I2C_ISR_ADDR = (1U << 3U);
    constexpr uint32_t I2C_ISR_NACKF = (1U << 4U);
    constexpr uint32_t I2C_ISR_STOPF = (1U << 5U);
    constexpr uint32_t I2C_ISR_TC = (1U << 6U);
    constexpr uint32_t I2C_ISR_TCR = (1U << 7U);
    constexpr uint32_t I2C_ISR_BERR = (1U << 8U);
    constexpr uint32_t I2C_ISR_ARLO = (1U << 9U);
    constexpr uint32_t I2C_ISR_OVR = (1U << 10U);
    constexpr uint32_t I2C_ISR_PECERR = (1U << 11U);
    constexpr uint32_t I2C_ISR_TIMEOUT = (1U << 12U);
    constexpr uint32_t I2C_ISR_ALERT = (1U << 13U);
    constexpr uint32_t I2C_ISR_BUSY = (1U << 15U);
    
    // ICR bits
    constexpr uint32_t I2C_ICR_NACKCF = (1U << 4U);
    constexpr uint32_t I2C_ICR_STOPCF = (1U << 5U);
    constexpr uint32_t I2C_ICR_BERRCF = (1U << 8U);
    constexpr uint32_t I2C_ICR_ARLOCF = (1U << 9U);
    constexpr uint32_t I2C_ICR_OVRCF = (1U << 10U);
    
    // RCC clock enable bits for I2C peripherals
    constexpr uint32_t RCC_APB1LENR_I2C1 = (1U << 21U);
    constexpr uint32_t RCC_APB1LENR_I2C2 = (1U << 22U);
    constexpr uint32_t RCC_APB1LENR_I2C3 = (1U << 23U);
    constexpr uint32_t RCC_APB4ENR_I2C4 = (1U << 4U);
    
    volatile uint32_t& RCC_APB1LENR = *reinterpret_cast<volatile uint32_t*>(0x58024464UL);
    volatile uint32_t& RCC_APB4ENR = *reinterpret_cast<volatile uint32_t*>(0x58024478UL);
}

I2cDriver::I2cDriver(uint32_t instance) 
    : instance_(instance), initialized_(false), config_() {
}

I2cDriver::~I2cDriver() {
    if (initialized_) {
        deinit();
    }
}

I2cDriver::Result I2cDriver::init(const I2cConfig& config) {
    if (initialized_) {
        return Result::ErrorBusy;
    }
    
    config_ = config;
    
    // Enable clock for the appropriate I2C instance
    switch (instance_) {
        case 1:
            RCC_APB1LENR |= RCC_APB1LENR_I2C1;
            break;
        case 2:
            RCC_APB1LENR |= RCC_APB1LENR_I2C2;
            break;
        case 3:
            RCC_APB1LENR |= RCC_APB1LENR_I2C3;
            break;
        case 4:
            RCC_APB4ENR |= RCC_APB4ENR_I2C4;
            break;
        default:
            return Result::ErrorInvalidParam;
    }
    
    volatile uint32_t* base = getBaseAddr();
    if (!base) {
        return Result::ErrorInvalidParam;
    }
    
    // Reset I2C peripheral
    base[I2C_CR1_OFF / 4U] |= I2C_CR1_SWRST;
    base[I2C_CR1_OFF / 4U] &= ~I2C_CR1_SWRST;
    
    // Configure timing registers for desired speed
    // STM32H7 I2C timing configuration depends on the clock frequency
    // For APB1=100MHz, I2C speed=100kHz:
    // PRESC=1, SCLDEL=4, SDADEL=2, SCLH=80, SCLL=100
    uint32_t timingr = 0;
    if (config_.speed <= 100000UL) {
        // Standard mode 100kHz
        timingr = (1U << 28U) |   // PRESC = 1
                  (4U << 20U) |   // SCLDEL = 4
                  (2U << 16U) |   // SDADEL = 2
                  (80U << 8U) |   // SCLH = 80
                  (100U << 0U);   // SCLL = 100
    } else {
        // Fast mode 400kHz
        timingr = (1U << 28U) |   // PRESC = 1
                  (3U << 20U) |   // SCLDEL = 3
                  (1U << 16U) |   // SDADEL = 1
                  (12U << 8U) |   // SCLH = 12
                  (25U << 0U);    // SCLL = 25
    }
    
    base[I2C_TIMINGR_OFF / 4U] = timingr;
    
    // Set own address
    base[I2C_OAR1_OFF / 4U] = (config_.ownAddress << 0U) | (1U << 15U);  // OA1EN
    
    // Enable I2C peripheral
    base[I2C_CR1_OFF / 4U] |= I2C_CR1_PE;
    
    initialized_ = true;
    return Result::Ok;
}

I2cDriver::Result I2cDriver::deinit() {
    if (!initialized_) {
        return Result::Ok;
    }
    
    volatile uint32_t* base = getBaseAddr();
    if (base) {
        base[I2C_CR1_OFF / 4U] &= ~I2C_CR1_PE;
    }
    
    initialized_ = false;
    return Result::Ok;
}

I2cDriver::Result I2cDriver::write(uint8_t deviceAddr, const uint8_t* data, uint16_t size, uint32_t timeoutMs) {
    if (!initialized_ || !data || size == 0) {
        return Result::ErrorInvalidParam;
    }
    
    volatile uint32_t* base = getBaseAddr();
    if (!base) return Result::ErrorInvalidParam;
    
    // Wait for bus not busy
    uint32_t timeout = timeoutMs * 1000;
    while (base[I2C_ISR_OFF / 4U] & I2C_ISR_BUSY) {
        if (--timeout == 0) return Result::ErrorTimeout;
    }
    
    // Clear any pending flags
    base[I2C_ICR_OFF / 4U] = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
    
    // Configure transfer: device address, write direction, NBYTES, auto end
    base[I2C_CR2_OFF / 4U] = ((uint32_t)deviceAddr << I2C_CR2_SADD_POS) |
                              ((uint32_t)size << I2C_CR2_NBYTES_POS) |
                              I2C_CR2_AUTOEND |
                              I2C_CR2_START;
    
    // Write data
    for (uint16_t i = 0; i < size; ++i) {
        timeout = timeoutMs * 1000;
        while (!(base[I2C_ISR_OFF / 4U] & I2C_ISR_TXIS)) {
            if (base[I2C_ISR_OFF / 4U] & I2C_ISR_NACKF) {
                base[I2C_ICR_OFF / 4U] = I2C_ICR_NACKCF;
                return Result::ErrorNack;
            }
            if (--timeout == 0) return Result::ErrorTimeout;
        }
        base[I2C_TXDR_OFF / 4U] = data[i];
    }
    
    // Wait for STOP
    timeout = timeoutMs * 1000;
    while (!(base[I2C_ISR_OFF / 4U] & I2C_ISR_STOPF)) {
        if (base[I2C_ISR_OFF / 4U] & I2C_ISR_NACKF) {
            base[I2C_ICR_OFF / 4U] = I2C_ICR_NACKCF;
            return Result::ErrorNack;
        }
        if (--timeout == 0) return Result::ErrorTimeout;
    }
    base[I2C_ICR_OFF / 4U] = I2C_ICR_STOPCF;
    
    return Result::Ok;
}

I2cDriver::Result I2cDriver::read(uint8_t deviceAddr, uint8_t* data, uint16_t size, uint32_t timeoutMs) {
    if (!initialized_ || !data || size == 0) {
        return Result::ErrorInvalidParam;
    }
    
    volatile uint32_t* base = getBaseAddr();
    if (!base) return Result::ErrorInvalidParam;
    
    // Wait for bus not busy
    uint32_t timeout = timeoutMs * 1000;
    while (base[I2C_ISR_OFF / 4U] & I2C_ISR_BUSY) {
        if (--timeout == 0) return Result::ErrorTimeout;
    }
    
    // Clear pending flags
    base[I2C_ICR_OFF / 4U] = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
    
    // Configure transfer: device address, read direction (R/W=1), NBYTES, auto end
    base[I2C_CR2_OFF / 4U] = ((uint32_t)deviceAddr << I2C_CR2_SADD_POS) |
                              I2C_CR2_RD_WRN |
                              ((uint32_t)size << I2C_CR2_NBYTES_POS) |
                              I2C_CR2_AUTOEND |
                              I2C_CR2_START;
    
    // Read data
    for (uint16_t i = 0; i < size; ++i) {
        timeout = timeoutMs * 1000;
        while (!(base[I2C_ISR_OFF / 4U] & I2C_ISR_RXNE)) {
            if (base[I2C_ISR_OFF / 4U] & I2C_ISR_NACKF) {
                base[I2C_ICR_OFF / 4U] = I2C_ICR_NACKCF;
                return Result::ErrorNack;
            }
            if (--timeout == 0) return Result::ErrorTimeout;
        }
        data[i] = (uint8_t)(base[I2C_RXDR_OFF / 4U] & 0xFFU);
    }
    
    // Wait for STOP
    timeout = timeoutMs * 1000;
    while (!(base[I2C_ISR_OFF / 4U] & I2C_ISR_STOPF)) {
        if (--timeout == 0) return Result::ErrorTimeout;
    }
    base[I2C_ICR_OFF / 4U] = I2C_ICR_STOPCF;
    
    return Result::Ok;
}

I2cDriver::Result I2cDriver::writeRegister(uint8_t deviceAddr, uint8_t reg, uint8_t value, uint32_t timeoutMs) {
    uint8_t data[2] = {reg, value};
    return write(deviceAddr, data, 2, timeoutMs);
}

I2cDriver::Result I2cDriver::readRegister(uint8_t deviceAddr, uint8_t reg, uint8_t& value, uint32_t timeoutMs) {
    // Write register address, then read back
    Result res = write(deviceAddr, &reg, 1, timeoutMs);
    if (res != Result::Ok) return res;
    return read(deviceAddr, &value, 1, timeoutMs);
}

I2cDriver::Result I2cDriver::writeRead(uint8_t deviceAddr, const uint8_t* writeData, uint16_t writeSize,
                                        uint8_t* readData, uint16_t readSize, uint32_t timeoutMs) {
    Result res = write(deviceAddr, writeData, writeSize, timeoutMs);
    if (res != Result::Ok) return res;
    return read(deviceAddr, readData, readSize, timeoutMs);
}

bool I2cDriver::isDeviceReady(uint8_t deviceAddr, uint32_t timeoutMs) {
    volatile uint32_t* base = getBaseAddr();
    if (!base) return false;
    
    // Wait for bus not busy
    uint32_t timeout = timeoutMs * 1000;
    while (base[I2C_ISR_OFF / 4U] & I2C_ISR_BUSY) {
        if (--timeout == 0) return false;
    }
    
    // Clear flags
    base[I2C_ICR_OFF / 4U] = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
    
    // Try to send address and check for NACK
    base[I2C_CR2_OFF / 4U] = ((uint32_t)deviceAddr << I2C_CR2_SADD_POS) |
                              (1U << I2C_CR2_NBYTES_POS) |
                              I2C_CR2_AUTOEND |
                              I2C_CR2_START;
    
    timeout = timeoutMs * 1000;
    while (!(base[I2C_ISR_OFF / 4U] & (I2C_ISR_STOPF | I2C_ISR_NACKF))) {
        if (--timeout == 0) return false;
    }
    
    bool ready = !(base[I2C_ISR_OFF / 4U] & I2C_ISR_NACKF);
    base[I2C_ICR_OFF / 4U] = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
    
    return ready;
}

volatile uint32_t* I2cDriver::getBaseAddr() const {
    switch (instance_) {
        case 1: return reinterpret_cast<volatile uint32_t*>(I2C1_BASE);
        case 2: return reinterpret_cast<volatile uint32_t*>(I2C2_BASE);
        case 3: return reinterpret_cast<volatile uint32_t*>(I2C3_BASE);
        case 4: return reinterpret_cast<volatile uint32_t*>(I2C4_BASE);
        default: return nullptr;
    }
}

} // namespace drone::drivers
