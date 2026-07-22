#include "drivers/mpu6050_driver.hpp"
#include "drivers/system_clock.hpp"

#include <cstdint>

namespace drone::drivers {

Mpu6050Driver::Mpu6050Driver(uint8_t address, uint8_t i2cInstance)
    : address_(address), i2c_(i2cInstance), initialized_(false),
      accelScale_(16384.0f), gyroScale_(131.0f) {
}

void Mpu6050Driver::init() {
    I2cConfig cfg;
    cfg.speed = 400000UL;  // 400kHz fast mode
    cfg.ownAddress = 0x00;
    
    if (i2c_.init(cfg) != I2cDriver::Result::Ok) {
        return;
    }
    
    // Reset device
    i2c_.writeRegister(address_, REG_PWR_MGMT_1, 0x80);
    SystemClock::delayMs(100);
    
    // Wake up device
    i2c_.writeRegister(address_, REG_PWR_MGMT_1, 0x00);
    SystemClock::delayMs(10);
    
    // Set clock source to gyro X-axis PLL
    i2c_.writeRegister(address_, REG_PWR_MGMT_1, 0x01);
    SystemClock::delayMs(10);
    
    // Set DLPF (Accel BW 184Hz, Gyro BW 188Hz)
    i2c_.writeRegister(address_, REG_CONFIG, 0x01);
    
    // Set sample rate divider (1kHz / (1+4) = 200Hz)
    i2c_.writeRegister(address_, REG_SMPLRT_DIV, 0x04);
    
    // Set accel range to ±2g (default)
    setAccelRange(0);
    
    // Set gyro range to ±250°/s (default)
    setGyroRange(0);
    
    // Enable I2C bypass for external devices
    i2c_.writeRegister(address_, REG_INT_PIN_CFG, 0x02);
    
    // Disable interrupts initially
    i2c_.writeRegister(address_, REG_INT_ENABLE, 0x00);
    
    initialized_ = true;
}

ImuReading Mpu6050Driver::read() {
    ImuReading reading{};
    if (!initialized_) return reading;
    
    uint8_t data[14];
    I2cDriver::Result res = i2c_.writeRead(address_, 
        (const uint8_t*)&REG_ACCEL_XOUT_H, 1, data, 14);
    
    if (res != I2cDriver::Result::Ok) {
        return reading;
    }
    
    // Parse accelerometer data (16-bit signed, big-endian)
    int16_t accelX = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    int16_t accelY = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    int16_t accelZ = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    
    // Parse temperature
    int16_t temp = (int16_t)(((uint16_t)data[6] << 8) | data[7]);
    
    // Parse gyroscope data
    int16_t gyroX = (int16_t)(((uint16_t)data[8] << 8) | data[9]);
    int16_t gyroY = (int16_t)(((uint16_t)data[10] << 8) | data[11]);
    int16_t gyroZ = (int16_t)(((uint16_t)data[12] << 8) | data[13]);
    
    // Convert to physical units
    reading.accelX = (float)accelX / accelScale_;
    reading.accelY = (float)accelY / accelScale_;
    reading.accelZ = (float)accelZ / accelScale_;
    
    reading.temperatureC = (float)temp / 340.0f + 36.53f;
    
    reading.gyroX = (float)gyroX / gyroScale_;
    reading.gyroY = (float)gyroY / gyroScale_;
    reading.gyroZ = (float)gyroZ / gyroScale_;
    
    return reading;
}

bool Mpu6050Driver::selfTest() {
    uint8_t whoAmI = 0;
    I2cDriver::Result res = i2c_.readRegister(address_, REG_WHO_AM_I, whoAmI);
    
    if (res != I2cDriver::Result::Ok) {
        return false;
    }
    
    // MPU6050 should return 0x68 or 0x98
    return (whoAmI == 0x68 || whoAmI == 0x98);
}

void Mpu6050Driver::setAccelRange(uint8_t range) {
    if (range > 3) range = 0;
    i2c_.writeRegister(address_, REG_ACCEL_CONFIG, range << 3);
    
    switch (range) {
        case 0: accelScale_ = 16384.0f; break;  // ±2g
        case 1: accelScale_ = 8192.0f;  break;  // ±4g
        case 2: accelScale_ = 4096.0f;  break;  // ±8g
        case 3: accelScale_ = 2048.0f;  break;  // ±16g
    }
}

void Mpu6050Driver::setGyroRange(uint8_t range) {
    if (range > 3) range = 0;
    i2c_.writeRegister(address_, REG_GYRO_CONFIG, range << 3);
    
    switch (range) {
        case 0: gyroScale_ = 131.0f;   break;  // ±250 °/s
        case 1: gyroScale_ = 65.5f;    break;  // ±500 °/s
        case 2: gyroScale_ = 32.75f;   break;  // ±1000 °/s
        case 3: gyroScale_ = 16.375f;  break;  // ±2000 °/s
    }
}

void Mpu6050Driver::setDigitalLowPassFilter(uint8_t mode) {
    if (mode > 6) mode = 0;
    i2c_.writeRegister(address_, REG_CONFIG, mode);
}

} // namespace drone::drivers
