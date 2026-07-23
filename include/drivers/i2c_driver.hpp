/**
 * @file    i2c_driver.hpp
 * @brief   STM32H7 I2C peripheral driver — register-level, 4 instances.
 *
 * @details Provides a complete I2C master driver with:
 *          - Support for I2C1, I2C2, I2C3, I2C4 (4 instances)
 *          - Standard (100 kHz) and Fast (400 kHz) mode
 *          - Blocking read/write with configurable timeout
 *          - Register-level access (no HAL dependency)
 *          - NACK detection, bus-busy detection, arbitration loss
 *          - Direct register write/read and combined writeRead transactions
 *          - Device presence check (isDeviceReady)
 *
 *          Hardware reference: STM32H743 RM0433 §45 (I2C).
 *
 * @ingroup drivers
 */

#pragma once

#include <cstdint>

namespace drone::drivers {

/**
 * @brief Configuration parameters for an I2C peripheral.
 */
struct I2cConfig {
    uint32_t speed = 100000UL;         ///< I2C clock speed in Hz (100 kHz or 400 kHz)
    uint32_t ownAddress = 0x00U;       ///< Own I2C address (7-bit, left-justified)
    bool useDma = false;               ///< Enable DMA for data transfer (not yet implemented)
};

/**
 * @defgroup i2c_driver I2C Master Driver
 * @brief    Register-level I2C master driver for STM32H7.
 *
 * ### Usage
 * @code{.cpp}
 *   I2cDriver i2c(1);                          // I2C1
 *   i2c.init({ .speed = 400000 });              // 400 kHz Fast Mode
 *   i2c.writeRegister(0x68, 0x6B, 0x00);        // Write MPU6050 PWR_MGMT_1
 *   uint8_t val;
 *   i2c.readRegister(0x68, 0x75, val);          // Read WHO_AM_I
 * @endcode
 *
 * @{
 */

class I2cDriver {
public:
    /// Results returned by all I2C operations.
    enum class Result {
        Ok,                 ///< Operation completed successfully.
        ErrorBusy,          ///< Bus is busy or peripheral occupied.
        ErrorNack,          ///< No acknowledge received from slave.
        ErrorTimeout,       ///< Operation timed out.
        ErrorInvalidParam   ///< Invalid instance or parameter.
    };

    /**
     * @brief  Construct an I2C driver instance.
     * @param  instance  I2C instance number (1–4).
     *         - 1 → I2C1  (0x40012000)
     *         - 2 → I2C2  (0x40013000)
     *         - 3 → I2C3  (0x40014000)
     *         - 4 → I2C4  (0x58001C00)
     */
    I2cDriver(uint32_t instance = 1);

    /// Destructor — calls deinit if active.
    ~I2cDriver();

    /**
     * @brief  Initialise the I2C peripheral.
     * @param  config  I2C configuration (speed, own address, DMA).
     * @return Result::Ok on success.
     */
    Result init(const I2cConfig& config = I2cConfig{});

    /// De-initialise the I2C peripheral and release resources.
    Result deinit();

    /**
     * @brief  Write data to a slave device (MEM+ADDR mode).
     * @param  deviceAddr  7-bit slave address.
     * @param  data        Pointer to data buffer.
     * @param  size        Number of bytes to write.
     * @param  timeoutMs   Timeout per byte in milliseconds.
     */
    Result write(uint8_t deviceAddr, const uint8_t* data, uint16_t size, uint32_t timeoutMs = 100);

    /**
     * @brief  Read data from a slave device (MEM+ADDR mode).
     * @param  deviceAddr  7-bit slave address.
     * @param  data        Pointer to receive buffer.
     * @param  size        Number of bytes to read.
     * @param  timeoutMs   Timeout per byte in milliseconds.
     */
    Result read(uint8_t deviceAddr, uint8_t* data, uint16_t size, uint32_t timeoutMs = 100);

    /**
     * @brief  Write a single byte to a register.
     * @param  deviceAddr  7-bit slave address.
     * @param  reg         Register address.
     * @param  value       Value to write.
     * @param  timeoutMs   Timeout in milliseconds.
     */
    Result writeRegister(uint8_t deviceAddr, uint8_t reg, uint8_t value, uint32_t timeoutMs = 100);

    /**
     * @brief  Read a single byte from a register.
     * @param  deviceAddr  7-bit slave address.
     * @param  reg         Register address.
     * @param[out] value   Read value.
     * @param  timeoutMs   Timeout in milliseconds.
     */
    Result readRegister(uint8_t deviceAddr, uint8_t reg, uint8_t& value, uint32_t timeoutMs = 100);

    /**
     * @brief  Combined write-then-read (for sensor register reads).
     * @param  deviceAddr  7-bit slave address.
     * @param  writeData   Data to write (typically register address).
     * @param  writeSize   Number of bytes to write.
     * @param[out] readData  Buffer for read data.
     * @param  readSize    Number of bytes to read.
     * @param  timeoutMs   Timeout in milliseconds.
     */
    Result writeRead(uint8_t deviceAddr, const uint8_t* writeData, uint16_t writeSize,
                     uint8_t* readData, uint16_t readSize, uint32_t timeoutMs = 100);

    /**
     * @brief  Check whether a slave device is ready on the bus.
     * @param  deviceAddr  7-bit slave address.
     * @param  timeoutMs   Timeout in milliseconds.
     * @return true if the device ACK'd its address.
     */
    bool isDeviceReady(uint8_t deviceAddr, uint32_t timeoutMs = 100);

private:
    uint32_t instance_;        ///< I2C instance number (1–4)
    bool initialized_;         ///< Whether the peripheral has been configured
    I2cConfig config_;         ///< Cached configuration

    /// @return Base address of the I2C instance registers.
    volatile uint32_t* getBaseAddr() const;

    // I2C peripheral base addresses (RM0433 §2.3)
    static constexpr uint32_t I2C1_BASE = 0x40012000UL;
    static constexpr uint32_t I2C2_BASE = 0x40013000UL;
    static constexpr uint32_t I2C3_BASE = 0x40014000UL;
    static constexpr uint32_t I2C4_BASE = 0x58001C00UL;
};

/** @} */  // end of i2c_driver group

} // namespace drone::drivers
