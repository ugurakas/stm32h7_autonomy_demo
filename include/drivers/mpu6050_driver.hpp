/**
 * @file    mpu6050_driver.hpp
 * @brief   MPU6050 IMU sensor driver — I2C-based 6-DOF accelerometer/gyroscope.
 *
 * @details Provides a complete driver for the InvenSense MPU6050:
 *          - I2C communication (register-level via I2cDriver)
 *          - Accelerometer: ±2g/±4g/±8g/±16g selectable range
 *          - Gyroscope: ±250/±500/±1000/±2000 °/s selectable range
 *          - Configurable digital low-pass filter (DLPF)
 *          - Built-in self-test per datasheet
 *          - Raw accelerometer, gyroscope, and temperature readings
 *          - Converts raw values to SI units (m/s², rad/s, °C)
 *
 *          Hardware: MPU6050 (InvenSense) — RM-MPU-6000A-00v6.6.
 *          Connected via I2C at address 0x68 (AD0 = 0) or 0x69 (AD0 = 1).
 *
 * @ingroup drivers
 */

#pragma once

#include <cstdint>
#include "drivers/i2c_driver.hpp"
#include "drivers/imu_sensor.hpp"

namespace drone::drivers {

/**
 * @defgroup mpu6050_driver MPU6050 IMU Driver
 * @brief    I2C-based 6-DOF accelerometer + gyroscope driver.
 *
 * ### Usage
 * @code{.cpp}
 *   Mpu6050Driver imu(0x68, 1);                  // I2C address 0x68, I2C1
 *   imu.init();                                   // Configure ±2g / ±250°/s
 *   bool ok = imu.selfTest();                     // Run self-test
 *   auto reading = imu.read();                    // Read all axes + temperature
 * @endcode
 *
 * @{
 */

class Mpu6050Driver : public ImuSensor {
public:
    /**
     * @brief  Construct an MPU6050 driver instance.
     * @param  address       I2C slave address (0x68 or 0x69).
     * @param  i2cInstance   I2C instance (1–4).
     */
    explicit Mpu6050Driver(uint8_t address = 0x68U, uint8_t i2cInstance = 1);

    /** @brief  Initialise the MPU6050: wake from sleep, configure ranges. */
    void init() override;

    /** @brief  Read all sensor data: accel (g), gyro (°/s), temp (°C). */
    ImuReading read() override;

    /** @brief  Run the MPU6050 self-test routine.
     *  @return true if self-test passes. */
    bool selfTest();

    /** @brief  Set accelerometer full-scale range.
     *  @param  range  0=±2g, 1=±4g, 2=±8g, 3=±16g */
    void setAccelRange(uint8_t range);

    /** @brief  Set gyroscope full-scale range.
     *  @param  range  0=±250, 1=±500, 2=±1000, 3=±2000 °/s */
    void setGyroRange(uint8_t range);

    /** @brief  Set digital low-pass filter (DLPF) mode.
     *  @param  mode  0–6 (see datasheet §4.12) */
    void setDigitalLowPassFilter(uint8_t mode);

private:
    uint8_t address_;         ///< I2C slave address (0x68 or 0x69)
    I2cDriver i2c_;           ///< I2C driver for communication
    bool initialized_;        ///< Whether the sensor has been initialised
    float accelScale_;        ///< Current accelerometer scale factor (g/LSB)
    float gyroScale_;         ///< Current gyroscope scale factor (°/s per LSB)

    // MPU6050 register map
    static constexpr uint8_t REG_SMPLRT_DIV     = 0x19;  ///< Sample rate divider
    static constexpr uint8_t REG_CONFIG          = 0x1A;  ///< DLPF configuration
    static constexpr uint8_t REG_GYRO_CONFIG     = 0x1B;  ///< Gyro full-scale range
    static constexpr uint8_t REG_ACCEL_CONFIG    = 0x1C;  ///< Accel full-scale range
    static constexpr uint8_t REG_ACCEL_CONFIG2   = 0x1D;  ///< Accel DLPF
    static constexpr uint8_t REG_INT_PIN_CFG     = 0x37;  ///< Interrupt pin config
    static constexpr uint8_t REG_INT_ENABLE      = 0x38;  ///< Interrupt enable
    static constexpr uint8_t REG_ACCEL_XOUT_H    = 0x3B;  ///< Accel X high byte
    static constexpr uint8_t REG_TEMP_OUT_H      = 0x41;  ///< Temperature high byte
    static constexpr uint8_t REG_GYRO_XOUT_H     = 0x43;  ///< Gyro X high byte
    static constexpr uint8_t REG_PWR_MGMT_1      = 0x6B;  ///< Power management 1
    static constexpr uint8_t REG_PWR_MGMT_2      = 0x6C;  ///< Power management 2
    static constexpr uint8_t REG_WHO_AM_I        = 0x75;  ///< Who Am I (0x68)
};

/** @} */  // end of mpu6050_driver group

} // namespace drone::drivers
