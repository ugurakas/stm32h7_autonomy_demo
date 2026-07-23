/**
 * @file    mpu6050_driver.cpp
 * @brief   Implementation of the MPU6050 I2C IMU driver.
 *
 * @details Implements the full MPU6050 driver:
 *          - I2C initialisation at 400 kHz (Fast Mode)
 *          - Device reset + wake-up sequence
 *          - Clock source selection (Gyro X-axis PLL)
 *          - DLPF configuration (BW: Accel 184 Hz, Gyro 188 Hz)
 *          - Sample rate divider (200 Hz output)
 *          - ±2g accelerometer / ±250°/s gyroscope (default ranges)
 *          - I2C bypass mode for external sensors
 *          - Burst read of all 14 registers (accel + temp + gyro)
 *          - Physical unit conversion (g, °/s, °C)
 *          - Self-test via WHO_AM_I register (expects 0x68/0x98)
 *          - Runtime range switching for accel and gyro
 *
 *          Hardware: InvenSense MPU6050 — RM-MPU-6000A-00v6.6.
 *
 * @ingroup drivers
 */

#include "drivers/mpu6050_driver.hpp"
#include "drivers/system_clock.hpp"

#include <cstdint>

namespace drone::drivers {

// ============================================================================
//  Constructor
// ============================================================================

Mpu6050Driver::Mpu6050Driver(uint8_t address, uint8_t i2cInstance)
    : address_(address), i2c_(i2cInstance), initialized_(false),
      accelScale_(16384.0f), gyroScale_(131.0f) {
}

// ============================================================================
//  Initialisation sequence (RM-MPU-6000A §4.29)
// ============================================================================

void Mpu6050Driver::init() {
    I2cConfig cfg;
    cfg.speed = 400000UL;   // Fast Mode 400 kHz
    cfg.ownAddress = 0x00;

    if (i2c_.init(cfg) != I2cDriver::Result::Ok) {
        return;
    }

    // Step 1: Reset device (register 0x6B, bit 7)
    i2c_.writeRegister(address_, REG_PWR_MGMT_1, 0x80);
    SystemClock::delayMs(100);

    // Step 2: Wake up (clear all bits)
    i2c_.writeRegister(address_, REG_PWR_MGMT_1, 0x00);
    SystemClock::delayMs(10);

    // Step 3: Select clock source = Gyro X-axis PLL (register 0x6B, bits 2:0 = 001)
    i2c_.writeRegister(address_, REG_PWR_MGMT_1, 0x01);
    SystemClock::delayMs(10);

    // Step 4: Set DLPF to mode 1 (Accel BW = 184 Hz, Gyro BW = 188 Hz)
    i2c_.writeRegister(address_, REG_CONFIG, 0x01);

    // Step 5: Set sample rate divider (1 kHz / (1 + 4) = 200 Hz)
    i2c_.writeRegister(address_, REG_SMPLRT_DIV, 0x04);

    // Step 6: Set default ranges
    setAccelRange(0);       // ±2g
    setGyroRange(0);        // ±250 °/s

    // Step 7: Enable I2C bypass (for external magnetometer, etc.)
    i2c_.writeRegister(address_, REG_INT_PIN_CFG, 0x02);

    // Step 8: Disable all interrupts initially
    i2c_.writeRegister(address_, REG_INT_ENABLE, 0x00);

    initialized_ = true;
}

// ============================================================================
//  Read — burst read of accel + temp + gyro (14 bytes)
// ============================================================================

ImuReading Mpu6050Driver::read() {
    ImuReading reading{};
    if (!initialized_) return reading;

    // Register auto-increment burst read from ACCEL_XOUT_H (0x3B)
    uint8_t data[14];
    I2cDriver::Result res = i2c_.writeRead(address_,
        (const uint8_t*)&REG_ACCEL_XOUT_H, 1, data, 14);

    if (res != I2cDriver::Result::Ok) {
        return reading;
    }

    // Parse big-endian two's complement 16-bit values
    int16_t accelX = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    int16_t accelY = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    int16_t accelZ = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    int16_t temp   = (int16_t)(((uint16_t)data[6] << 8) | data[7]);
    int16_t gyroX  = (int16_t)(((uint16_t)data[8] << 8) | data[9]);
    int16_t gyroY  = (int16_t)(((uint16_t)data[10] << 8) | data[11]);
    int16_t gyroZ  = (int16_t)(((uint16_t)data[12] << 8) | data[13]);

    // Convert to physical units
    reading.accelX = (float)accelX / accelScale_;           // g
    reading.accelY = (float)accelY / accelScale_;
    reading.accelZ = (float)accelZ / accelScale_;

    reading.temperatureC = (float)temp / 340.0f + 36.53f;   // °C

    reading.gyroX = (float)gyroX / gyroScale_;              // °/s
    reading.gyroY = (float)gyroY / gyroScale_;
    reading.gyroZ = (float)gyroZ / gyroScale_;

    return reading;
}

// ============================================================================
//  Self-test — verify WHO_AM_I register
// ============================================================================

bool Mpu6050Driver::selfTest() {
    uint8_t whoAmI = 0;
    I2cDriver::Result res = i2c_.readRegister(address_, REG_WHO_AM_I, whoAmI);

    if (res != I2cDriver::Result::Ok) {
        return false;
    }

    // MPU6050 should return 0x68 (or 0x98 for MPU9150 in bypass)
    return (whoAmI == 0x68 || whoAmI == 0x98);
}

// ============================================================================
//  Range configuration
// ============================================================================

void Mpu6050Driver::setAccelRange(uint8_t range) {
    if (range > 3) range = 0;
    i2c_.writeRegister(address_, REG_ACCEL_CONFIG, range << 3);

    switch (range) {
        case 0: accelScale_ = 16384.0f; break;   // ±2g
        case 1: accelScale_ = 8192.0f;  break;   // ±4g
        case 2: accelScale_ = 4096.0f;  break;   // ±8g
        case 3: accelScale_ = 2048.0f;  break;   // ±16g
    }
}

void Mpu6050Driver::setGyroRange(uint8_t range) {
    if (range > 3) range = 0;
    i2c_.writeRegister(address_, REG_GYRO_CONFIG, range << 3);

    switch (range) {
        case 0: gyroScale_ = 131.0f;   break;    // ±250 °/s
        case 1: gyroScale_ = 65.5f;    break;    // ±500 °/s
        case 2: gyroScale_ = 32.75f;   break;    // ±1000 °/s
        case 3: gyroScale_ = 16.375f;  break;    // ±2000 °/s
    }
}

void Mpu6050Driver::setDigitalLowPassFilter(uint8_t mode) {
    if (mode > 6) mode = 0;
    i2c_.writeRegister(address_, REG_CONFIG, mode);
}

} // namespace drone::drivers
