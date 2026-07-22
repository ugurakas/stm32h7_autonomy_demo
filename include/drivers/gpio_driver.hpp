#pragma once

#include <cstdint>

namespace drone::drivers {

struct GpioConfig {
    uint32_t pin = 0;
    uint32_t mode = 0;      // 0=Input, 1=Output, 2=AF, 3=Analog
    uint32_t pull = 0;      // 0=No pull, 1=Pull-up, 2=Pull-down
    uint32_t speed = 3;     // 0=Low, 1=Medium, 2=High, 3=Very High
    uint32_t outputType = 0; // 0=Push-pull, 1=Open-drain
    uint32_t alternate = 0;  // AF number (0-15)
};

enum class GpioPort : uint32_t {
    PortA = 0,
    PortB = 1,
    PortC = 2,
    PortD = 3,
    PortE = 4,
    PortF = 5,
    PortG = 6,
    PortH = 7,
    PortI = 8,
    PortJ = 9,
    PortK = 10
};

class GpioDriver {
public:
    GpioDriver(GpioPort port, uint32_t pin);
    ~GpioDriver() = default;
    
    void configure(const GpioConfig& config);
    void set();
    void reset();
    void toggle();
    void write(bool value);
    bool read() const;
    
    // Static methods for quick operations
    static void writePin(GpioPort port, uint32_t pin, bool value);
    static bool readPin(GpioPort port, uint32_t pin);

private:
    GpioPort port_;
    uint32_t pin_;
    
    volatile uint32_t* getBaseAddr() const;
    void enableClock();
    
    static constexpr uint32_t GPIOA_BASE = 0x58020000UL;
    static constexpr uint32_t GPIOB_BASE = 0x58020400UL;
    static constexpr uint32_t GPIOC_BASE = 0x58020800UL;
    static constexpr uint32_t GPIOD_BASE = 0x58020C00UL;
    static constexpr uint32_t GPIOE_BASE = 0x58021000UL;
    static constexpr uint32_t GPIOF_BASE = 0x58021400UL;
    static constexpr uint32_t GPIOG_BASE = 0x58021800UL;
    static constexpr uint32_t GPIOH_BASE = 0x58021C00UL;
    static constexpr uint32_t GPIOI_BASE = 0x58022000UL;
    static constexpr uint32_t GPIOJ_BASE = 0x58022400UL;
    static constexpr uint32_t GPIOK_BASE = 0x58022800UL;
};

// LED and status indicator definitions for Nucleo-H743ZI
namespace board {
    // User LED (LED1) on PA5
    constexpr GpioPort LED_PORT = GpioPort::PortA;
    constexpr uint32_t LED_PIN = 5;
    
    // User button (B1) on PC13
    constexpr GpioPort BTN_PORT = GpioPort::PortC;
    constexpr uint32_t BTN_PIN = 13;
    
    // Status LEDs
    constexpr uint32_t STATUS_LED_PINS[] = {5, 6, 7}; // PA5, PA6, PA7
    
    inline void initStatusLed() {
        GpioDriver led(LED_PORT, LED_PIN);
        GpioConfig cfg;
        cfg.mode = 1; // Output
        cfg.outputType = 0; // Push-pull
        cfg.pull = 0;
        cfg.speed = 1;
        led.configure(cfg);
    }
    
    inline void setStatusLed(bool on) {
        GpioDriver::writePin(LED_PORT, LED_PIN, on);
    }
    
    inline void toggleStatusLed() {
        GpioDriver led(LED_PORT, LED_PIN);
        led.toggle();
    }
    
    inline bool readButton() {
        return GpioDriver::readPin(BTN_PORT, BTN_PIN);
    }
}

} // namespace drone::drivers
