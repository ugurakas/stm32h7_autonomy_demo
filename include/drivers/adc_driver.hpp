#pragma once

#include <cstdint>

namespace drone::drivers {

struct AdcConfig {
    uint32_t resolution = 12;        // 12-bit
    uint32_t samplingCycles = 15;    // 15 ADC clock cycles
    bool continuousConv = false;
    bool useDma = false;
};

class AdcDriver {
public:
    enum class Result {
        Ok,
        ErrorBusy,
        ErrorTimeout,
        ErrorInvalidParam
    };

    AdcDriver(uint32_t instance = 1);  // ADC1-3
    ~AdcDriver();

    Result init(const AdcConfig& config = AdcConfig{});
    Result deinit();
    
    Result startConversion();
    Result stopConversion();
    
    uint32_t readRaw();
    float readVoltage(float vref = 3.3f);
    
    // For battery monitoring
    float readBatteryVoltage(float voltageDividerRatio = 2.0f);

private:
    uint32_t instance_;
    bool initialized_;
    AdcConfig config_;
    
    volatile uint32_t* getBaseAddr() const;
    void enableClock();
    
    static constexpr uint32_t ADC1_BASE = 0x40022000UL;
    static constexpr uint32_t ADC2_BASE = 0x40022100UL;
    static constexpr uint32_t ADC3_BASE = 0x40022200UL;
};

} // namespace drone::drivers
