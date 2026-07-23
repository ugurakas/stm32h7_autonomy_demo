/**
 * @file    gpio_driver.hpp
 * @brief   STM32H7 GPIO driver — register-level pin control.
 *
 * @details Provides complete GPIO functionality:
 *          - 11 GPIO ports (A–K) with full register access
 *          - Configurable mode (input, output, alternate function, analog)
 *          - Pull-up/pull-down, speed, output type, alternate function selection
 *          - Atomic set/reset/toggle via BSRR register
 *          - Static helper functions for fast pin operations
 *          - Built-in board definitions for Nucleo-H743ZI LED+button
 *
 *          Hardware reference: STM32H743 RM0433 §10 (GPIO).
 *          Register-level access — no HAL dependency.
 *
 * @ingroup drivers
 */

#pragma once

#include <cstdint>

namespace drone::drivers {

/**
 * @brief GPIO pin configuration structure.
 */
struct GpioConfig {
    uint32_t pin = 0;          ///< Pin number (0–15)
    uint32_t mode = 0;         ///< 0=Input, 1=Output, 2=AF, 3=Analog
    uint32_t pull = 0;         ///< 0=No pull, 1=Pull-up, 2=Pull-down
    uint32_t speed = 3;        ///< 0=Low, 1=Medium, 2=High, 3=Very High
    uint32_t outputType = 0;   ///< 0=Push-pull, 1=Open-drain
    uint32_t alternate = 0;    ///< Alternate function number (0–15)
};

/**
 * @brief GPIO port selection.
 */
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

/**
 * @defgroup gpio_driver GPIO Driver
 * @brief    Register-level GPIO driver for STM32H7.
 *
 * ### Usage
 * @code{.cpp}
 *   GpioDriver led(GpioPort::PortA, 5);       // PA5 = Nucleo LED
 *   led.configure({ .mode = 1 });             // Output, push-pull
 *   led.write(true);                           // Turn on
 *   led.toggle();                              // Toggle
 *   bool btn = GpioDriver::readPin(GpioPort::PortC, 13);  // PC13 button
 * @endcode
 *
 * @{
 */

class GpioDriver {
public:
    /**
     * @brief  Construct a GPIO pin driver.
     * @param  port  GPIO port (PortA–PortK).
     * @param  pin   Pin number (0–15).
     */
    GpioDriver(GpioPort port, uint32_t pin);

    /// Destructor (trivial).
    ~GpioDriver() = default;

    /** @brief  Configure the pin mode, pull, speed, output type, AF. */
    void configure(const GpioConfig& config);

    /// Set pin high (using BSRR atomic set).
    void set();

    /// Reset pin low (using BSRR atomic reset).
    void reset();

    /// Toggle the pin output value.
    void toggle();

    /// Write a boolean value to the pin.
    void write(bool value);

    /** @brief  Read the current input state of the pin.
     *  @return true if pin is high. */
    bool read() const;

    /** @brief  Static helper: atomically write a pin. */
    static void writePin(GpioPort port, uint32_t pin, bool value);

    /** @brief  Static helper: atomically read a pin. */
    static bool readPin(GpioPort port, uint32_t pin);

private:
    GpioPort port_;       ///< GPIO port
    uint32_t pin_;         ///< Pin number

    /// @return Base address of the GPIO port registers.
    volatile uint32_t* getBaseAddr() const;

    /// Enable the clock for this GPIO port (RCC_AHB4ENR).
    void enableClock();

    // GPIO base addresses (RM0433 §2.3)
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

// ---------------------------------------------------------------------------
//  Board-specific definitions for Nucleo-H743ZI
// ---------------------------------------------------------------------------

namespace board {
    /// User LED (LD1) on PA5
    constexpr GpioPort LED_PORT = GpioPort::PortA;
    constexpr uint32_t LED_PIN = 5;

    /// User button (B1) on PC13 (pull-up, 0 = pressed)
    constexpr GpioPort BTN_PORT = GpioPort::PortC;
    constexpr uint32_t BTN_PIN = 13;

    /// Status LED pins (PA5, PA6, PA7)
    constexpr uint32_t STATUS_LED_PINS[] = {5, 6, 7};

    /// Initialise the user LED as a push-pull output.
    inline void initStatusLed() {
        GpioDriver led(LED_PORT, LED_PIN);
        GpioConfig cfg;
        cfg.mode = 1;
        cfg.outputType = 0;
        cfg.pull = 0;
        cfg.speed = 1;
        led.configure(cfg);
    }

    /// Set the user LED on/off.
    inline void setStatusLed(bool on) {
        GpioDriver::writePin(LED_PORT, LED_PIN, on);
    }

    /// Toggle the user LED.
    inline void toggleStatusLed() {
        GpioDriver led(LED_PORT, LED_PIN);
        led.toggle();
    }

    /// Read the user button (true = not pressed, false = pressed).
    inline bool readButton() {
        return GpioDriver::readPin(BTN_PORT, BTN_PIN);
    }
}

/** @} */  // end of gpio_driver group

} // namespace drone::drivers
