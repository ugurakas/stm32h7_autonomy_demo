/**
 * @file    adc_driver.hpp
 * @brief   STM32H7 ADC peripheral driver — register-level, 3 instances.
 *
 * @details Provides a complete ADC driver for battery voltage monitoring:
 *          - Support for ADC1, ADC2, ADC3 (3 instances)
 *          - 12-bit resolution with configurable sampling time
 *          - Single-shot and continuous conversion modes
 *          - Optional DMA support for multi-channel scanning
 *          - Built-in battery voltage reading with voltage-divider ratio
 *          - Calibration sequence for offset/gain correction
 *
 *          Hardware reference: STM32H743 RM0433 §27 (ADC).
 *          Register-level access — no HAL dependency.
 *
 * @ingroup drivers
 */

#pragma once

#include <cstdint>

namespace drone::drivers {

/**
 * @brief ADC configuration parameters.
 */
struct AdcConfig {
    uint32_t resolution = 12;         ///< Resolution in bits: 6, 8, 10, 12, 14, 16
    uint32_t samplingCycles = 15;     ///< Sampling cycles: 1.5 to 810.5
    bool continuousConv = false;      ///< Continuous conversion mode
    bool useDma = false;              ///< Enable DMA for regular group
};

/**
 * @defgroup adc_driver ADC Driver
 * @brief    Register-level ADC driver for STM32H7 battery/sensor reading.
 *
 * ### Usage
 * @code{.cpp}
 *   AdcDriver adc(1);                          // ADC1
 *   adc.init();                                 // Default 12-bit, single-shot
 *   float vbat = adc.readBatteryVoltage(2.0f);  // Read with /2 divider
 * @endcode
 *
 * @{
 */

class AdcDriver {
public:
    /// Results returned by ADC operations.
    enum class Result {
        Ok,                 ///< Operation completed successfully.
        ErrorBusy,          ///< ADC is busy with previous conversion.
        ErrorTimeout,       ///< Operation timed out.
        ErrorInvalidParam   ///< Invalid instance or parameter.
    };

    /**
     * @brief  Construct an ADC driver instance.
     * @param  instance  ADC instance number (1–3).
     *         - 1 → ADC1 (0x40022000)
     *         - 2 → ADC2 (0x40022100)
     *         - 3 → ADC3 (0x40022200)
     */
    AdcDriver(uint32_t instance = 1);

    /// Destructor — calls deinit if active.
    ~AdcDriver();

    /**
     * @brief  Initialise the ADC peripheral.
     * @param  config  ADC configuration (resolution, sampling, etc.).
     * @return Result::Ok on success.
     */
    Result init(const AdcConfig& config = AdcConfig{});

    /// De-initialise the ADC peripheral.
    Result deinit();

    /// Start a conversion (single-shot mode).
    Result startConversion();

    /// Stop an ongoing conversion.
    Result stopConversion();

    /** @brief  Read raw ADC conversion result.
     *  @return 16-bit raw ADC value (right-aligned). */
    uint32_t readRaw();

    /** @brief  Convert raw ADC reading to voltage.
     *  @param  vref  Reference voltage in volts (default 3.3 V).
     *  @return Voltage at the ADC pin. */
    float readVoltage(float vref = 3.3f);

    /** @brief  Read battery voltage through a voltage divider.
     *  @param  voltageDividerRatio  Divider factor (e.g. 2.0 for /2).
     *  @return Actual battery voltage. */
    float readBatteryVoltage(float voltageDividerRatio = 2.0f);

private:
    uint32_t instance_;        ///< ADC instance number (1–3)
    bool initialized_;         ///< Whether the peripheral is configured
    AdcConfig config_;         ///< Cached configuration

    /// @return Base address of the ADC instance registers.
    volatile uint32_t* getBaseAddr() const;

    void enableClock();

    // ADC base addresses (RM0433 §2.3)
    static constexpr uint32_t ADC1_BASE = 0x40022000UL;
    static constexpr uint32_t ADC2_BASE = 0x40022100UL;
    static constexpr uint32_t ADC3_BASE = 0x40022200UL;
};

/** @} */  // end of adc_driver group

} // namespace drone::drivers
