#pragma once

#include <cstdint>
#include "drivers/i2c_driver.hpp"
#include "drivers/imu_sensor.hpp"

namespace drone::drivers {

class Mpu6050Driver : public ImuSensor {
public:
    explicit Mpu6050Driver(uint8_t address = 0x68U, uint8_t i2cInstance = 1);
    void init() override;
    ImuReading read() override;
    bool selfTest();
    void setAccelRange(uint8_t range);
    void setGyroRange(uint8_t range);
    void setDigitalLowPassFilter(uint8_t mode);

private:
    uint8_t address_;
    I2cDriver i2c_;
    bool initialized_;
    float accelScale_;
    float gyroScale_;
    
    static constexpr uint8_t REG_SMPLRT_DIV = 0x19;
    static constexpr uint8_t REG_CONFIG = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;
    static constexpr uint8_t REG_INT_PIN_CFG = 0x37;
    static constexpr uint8_t REG_INT_ENABLE = 0x38;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t REG_TEMP_OUT_H = 0x41;
    static constexpr uint8_t REG_GYRO_XOUT_H = 0x43;
    static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
    static constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;
    static constexpr uint8_t REG_WHO_AM_I = 0x75;
};

} // namespace drone::drivers
