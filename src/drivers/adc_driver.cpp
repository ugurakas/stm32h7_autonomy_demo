/**
 * @file    adc_driver.cpp
 * @brief   Implementation of the STM32H7 ADC driver.
 *
 * @details Implements register-level ADC operations for STM32H7:
 *          - ADC1–ADC3 with clock enable via RCC_AHB4ENR
 *          - Voltage regulator enable, calibration sequence
 *          - 6-/8-/10-/12-bit resolution selection
 *          - Single-shot and continuous conversion modes
 *          - Battery voltage and VRefInt sensor channel enable
 *          - Raw read, voltage conversion, and battery voltage
 *
 *          Register addresses conform to RM0433 §27.
 *          ADC123_COMMON base = 0x40022100 for VBAT enable.
 *
 * @ingroup drivers
 */

#include "drivers/adc_driver.hpp"

#include <cstdint>

namespace drone::drivers {

// ---------------------------------------------------------------------------
//  ADC register offsets (RM0433 §27.6)
// ---------------------------------------------------------------------------
namespace {
    constexpr uint32_t ADC_CR_OFF     = 0x08U;   ///< Control register
    constexpr uint32_t ADC_CFGR_OFF   = 0x0CU;   ///< Configuration register
    constexpr uint32_t ADC_SMPR1_OFF  = 0x14U;   ///< Sampling time register 1
    constexpr uint32_t ADC_SMPR2_OFF  = 0x18U;   ///< Sampling time register 2
    constexpr uint32_t ADC_TR1_OFF    = 0x20U;   ///< Watchdog threshold register 1
    constexpr uint32_t ADC_DR_OFF     = 0x40U;   ///< Data register
    constexpr uint32_t ADC_CSR_OFF    = 0x00U;   ///< Common status register
    constexpr uint32_t ADC_CCR_OFF    = 0x04U;   ///< Common configuration register

    // ---- CR bits ----
    constexpr uint32_t ADC_CR_ADEN      = (1U << 0U);     ///< ADC enable
    constexpr uint32_t ADC_CR_ADSTART   = (1U << 2U);     ///< Start conversion
    constexpr uint32_t ADC_CR_ADSTP     = (1U << 4U);     ///< Stop conversion
    constexpr uint32_t ADC_CR_ADVREGE   = (1U << 28U);    ///< Voltage regulator enable
    constexpr uint32_t ADC_CR_ADCAL     = (1U << 29U);    ///< Calibration
    constexpr uint32_t ADC_CR_DEEPPWD   = (1U << 30U);    ///< Deep-power-down

    // ---- CSR bits ----
    constexpr uint32_t ADC_CSR_ADRDY    = (1U << 0U);     ///< ADC ready
    constexpr uint32_t ADC_CSR_EOC      = (1U << 2U);     ///< End of conversion

    // ---- CFGR bits ----
    constexpr uint32_t ADC_CFGR_RES_MASK  = (3U << 3U);    ///< Resolution bitfield mask
    constexpr uint32_t ADC_CFGR_RES_12    = (0U << 3U);    ///< 12-bit
    constexpr uint32_t ADC_CFGR_RES_10    = (1U << 3U);    ///< 10-bit
    constexpr uint32_t ADC_CFGR_RES_8     = (2U << 3U);    ///< 8-bit
    constexpr uint32_t ADC_CFGR_RES_6     = (3U << 3U);    ///< 6-bit
    constexpr uint32_t ADC_CFGR_CONT      = (1U << 13U);   ///< Continuous conversion
    constexpr uint32_t ADC_CFGR_DMAEN     = (1U << 0U);    ///< DMA enable
    constexpr uint32_t ADC_CFGR_DMACFG    = (1U << 1U);    ///< DMA config (circular)

    // ---- CCR bits (ADC123_COMMON) ----
    constexpr uint32_t ADC_CCR_VSENSESEL  = (1U << 1U);    ///< Internal temperature sensor
    constexpr uint32_t ADC_CCR_VBATSEL    = (1U << 2U);    ///< VBAT battery channel

    // ---- RCC clock enables (RM0433 §5.8) ----
    constexpr uint32_t RCC_AHB4ENR_ADC12  = (1U << 5U);    ///< ADC1 and ADC2 on AHB4
    constexpr uint32_t RCC_AHB4ENR_ADC3   = (1U << 6U);    ///< ADC3 on AHB4

    volatile uint32_t& RCC_AHB4ENR = *reinterpret_cast<volatile uint32_t*>(0x580244E0UL);
}

// ============================================================================
//  Constructor / Destructor
// ============================================================================

AdcDriver::AdcDriver(uint32_t instance)
    : instance_(instance), initialized_(false), config_() {
}

AdcDriver::~AdcDriver() {
    if (initialized_) {
        deinit();
    }
}

// ============================================================================
//  init — Full ADC initialisation sequence
// ============================================================================

AdcDriver::Result AdcDriver::init(const AdcConfig& config) {
    if (initialized_) return Result::ErrorBusy;

    config_ = config;
    enableClock();

    volatile uint32_t* base = getBaseAddr();
    if (!base) return Result::ErrorInvalidParam;

    // Step 1: Enable ADC voltage regulator
    base[ADC_CR_OFF / 4U] |= ADC_CR_ADVREGE;
    volatile uint32_t wait = 10000;
    while (--wait) { }   // tADCVREG_STUP ≈ 10 µs

    // Step 2: Calibrate (offset + linearity)
    base[ADC_CR_OFF / 4U] |= ADC_CR_ADCAL;
    while (base[ADC_CR_OFF / 4U] & ADC_CR_ADCAL) { }

    // Step 3: Enable ADC
    base[ADC_CR_OFF / 4U] |= ADC_CR_ADEN;
    while (!(base[ADC_CSR_OFF / 4U] & ADC_CSR_ADRDY)) { }

    // Step 4: Set resolution
    uint32_t cfgr = base[ADC_CFGR_OFF / 4U] & ~ADC_CFGR_RES_MASK;
    switch (config_.resolution) {
        case 10: cfgr |= ADC_CFGR_RES_10; break;
        case 8:  cfgr |= ADC_CFGR_RES_8;  break;
        case 6:  cfgr |= ADC_CFGR_RES_6;  break;
        default: cfgr |= ADC_CFGR_RES_12; break;
    }
    if (config_.continuousConv) cfgr |= ADC_CFGR_CONT;
    if (config_.useDma)         cfgr |= ADC_CFGR_DMAEN;
    base[ADC_CFGR_OFF / 4U] = cfgr;

    // Step 5: Enable battery sensor and temperature sensor in common regs
    volatile uint32_t* common = reinterpret_cast<volatile uint32_t*>(0x40022100UL);
    common[ADC_CCR_OFF / 4U] |= ADC_CCR_VBATSEL | ADC_CCR_VSENSESEL;

    initialized_ = true;
    return Result::Ok;
}

AdcDriver::Result AdcDriver::deinit() {
    if (!initialized_) return Result::Ok;
    volatile uint32_t* base = getBaseAddr();
    if (base) {
        base[ADC_CR_OFF / 4U] = ADC_CR_DEEPPWD;  // Deep-power-down
    }
    initialized_ = false;
    return Result::Ok;
}

AdcDriver::Result AdcDriver::startConversion() {
    volatile uint32_t* base = getBaseAddr();
    if (!base) return Result::ErrorInvalidParam;
    base[ADC_CR_OFF / 4U] |= ADC_CR_ADSTART;
    return Result::Ok;
}

AdcDriver::Result AdcDriver::stopConversion() {
    volatile uint32_t* base = getBaseAddr();
    if (!base) return Result::ErrorInvalidParam;
    base[ADC_CR_OFF / 4U] |= ADC_CR_ADSTP;
    return Result::Ok;
}

uint32_t AdcDriver::readRaw() {
    volatile uint32_t* base = getBaseAddr();
    if (!base) return 0;

    // Start single conversion
    base[ADC_CR_OFF / 4U] |= ADC_CR_ADSTART;

    // Wait for end of conversion
    while (!(base[ADC_CSR_OFF / 4U] & ADC_CSR_EOC)) { }

    return base[ADC_DR_OFF / 4U] & 0xFFFFU;
}

float AdcDriver::readVoltage(float vref) {
    uint32_t raw = readRaw();
    float maxValue = (config_.resolution == 12) ? 4095.0f :
                     (config_.resolution == 10) ? 1023.0f :
                     (config_.resolution == 8)  ? 255.0f : 63.0f;
    return (static_cast<float>(raw) / maxValue) * vref;
}

float AdcDriver::readBatteryVoltage(float voltageDividerRatio) {
    return readVoltage() * voltageDividerRatio;
}

volatile uint32_t* AdcDriver::getBaseAddr() const {
    switch (instance_) {
        case 1: return reinterpret_cast<volatile uint32_t*>(ADC1_BASE);
        case 2: return reinterpret_cast<volatile uint32_t*>(ADC2_BASE);
        case 3: return reinterpret_cast<volatile uint32_t*>(ADC3_BASE);
        default: return nullptr;
    }
}

void AdcDriver::enableClock() {
    RCC_AHB4ENR |= (instance_ <= 2) ? RCC_AHB4ENR_ADC12 : RCC_AHB4ENR_ADC3;
}

} // namespace drone::drivers
