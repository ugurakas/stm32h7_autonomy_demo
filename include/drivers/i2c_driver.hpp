#pragma once

#include <cstdint>

namespace drone::drivers {

struct I2cConfig {
    uint32_t speed = 100000UL;         // 100 kHz standard mode
    uint32_t ownAddress = 0x00U;
    bool useDma = false;
};

class I2cDriver {
public:
    enum class Result {
        Ok,
        ErrorBusy,
        ErrorNack,
        ErrorTimeout,
        ErrorInvalidParam
    };

    I2cDriver(uint32_t instance = 1);  // 1 = I2C1, 2 = I2C2, 3 = I2C3, 4 = I2C4
    ~I2cDriver();

    Result init(const I2cConfig& config = I2cConfig{});
    Result deinit();
    
    Result write(uint8_t deviceAddr, const uint8_t* data, uint16_t size, uint32_t timeoutMs = 100);
    Result read(uint8_t deviceAddr, uint8_t* data, uint16_t size, uint32_t timeoutMs = 100);
    
    Result writeRegister(uint8_t deviceAddr, uint8_t reg, uint8_t value, uint32_t timeoutMs = 100);
    Result readRegister(uint8_t deviceAddr, uint8_t reg, uint8_t& value, uint32_t timeoutMs = 100);
    
    Result writeRead(uint8_t deviceAddr, const uint8_t* writeData, uint16_t writeSize,
                     uint8_t* readData, uint16_t readSize, uint32_t timeoutMs = 100);
    
    bool isDeviceReady(uint8_t deviceAddr, uint32_t timeoutMs = 100);

private:
    uint32_t instance_;
    bool initialized_;
    I2cConfig config_;
    
    // I2C register map (for direct register access)
    volatile uint32_t* getBaseAddr() const;
    
    static constexpr uint32_t I2C1_BASE = 0x40012000UL;
    static constexpr uint32_t I2C2_BASE = 0x40013000UL;
    static constexpr uint32_t I2C3_BASE = 0x40014000UL;
    static constexpr uint32_t I2C4_BASE = 0x58001C00UL;
};

} // namespace drone::drivers
