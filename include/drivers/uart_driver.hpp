/**
 * @file    uart_driver.hpp
 * @brief   STM32H7 UART/USART driver — register-level, 8 instances with ring buffers.
 *
 * @details Provides a full UART driver with:
 *          - Support for USART1-6, UART7-8 (8 instances)
 *          - Configurable baud rate, word length, stop bits, parity
 *          - Interrupt-driven RX with circular ring buffer
 *          - Blocking TX with timeout
 *          - Optional DMA support (RX/TX)
 *          - Polled receive and buffer read functions
 *          - Error detection: overrun, framing, parity, noise
 *
 *          Hardware reference: STM32H743 RM0433 §47 (USART).
 *          Register-level access — no HAL dependency.
 *
 * @ingroup drivers
 */

#pragma once

#include <cstdint>

namespace drone::drivers {

/**
 * @brief UART configuration parameters.
 */
struct UartConfig {
    uint32_t baudrate = 115200UL;     ///< Serial baud rate (bps)
    uint32_t wordLength = 8;          ///< Data bits: 7, 8, or 9
    uint32_t stopBits = 1;            ///< Stop bits: 1 or 2
    bool parity = false;              ///< Enable parity bit
    bool parityEven = false;          ///< true = even parity, false = odd
    bool useRxDma = false;            ///< Enable RX DMA stream
    bool useTxDma = false;            ///< Enable TX DMA stream
    uint16_t rxBufferSize = 256;      ///< RX ring buffer size (bytes)
};

/**
 * @defgroup uart_driver UART Driver
 * @brief    Register-level UART/USART driver for STM32H7.
 *
 * ### Usage
 * @code{.cpp}
 *   UartDriver uart(3);                         // USART3
 *   uart.init({ .baudrate = 115200 });           // 115200 8N1
 *   uart.transmit((uint8_t*)"Hello\r\n", 7);     // Blocking TX
 *   uint8_t buf[64];
 *   uint16_t n = uart.readRxBuffer(buf, 64);     // Read RX buffer
 * @endcode
 *
 * @{
 */

class UartDriver {
public:
    /// Results returned by UART operations.
    enum class Result {
        Ok,                 ///< Operation completed successfully.
        ErrorBusy,          ///< Peripheral is already initialised.
        ErrorTimeout,       ///< Operation timed out.
        ErrorOverrun,       ///< RX overrun error detected.
        ErrorFraming,       ///< Framing error detected.
        ErrorParity,        ///< Parity error detected.
        ErrorInvalidParam   ///< Invalid instance or parameter.
    };

    /**
     * @brief  Construct a UART driver instance.
     * @param  instance  UART instance number (1–8).
     *         - 1 → USART1 (0x40011000)
     *         - 2 → USART2 (0x40004400)
     *         - 3 → USART3 (0x40004800)
     *         - 4 → UART4  (0x40004C00)
     *         - 5 → UART5  (0x40005000)
     *         - 6 → USART6 (0x40011400)
     *         - 7 → UART7  (0x40007800)
     *         - 8 → UART8  (0x40007C00)
     */
    UartDriver(uint32_t instance = 1);

    /// Destructor — calls deinit and frees the RX buffer.
    ~UartDriver();

    /**
     * @brief  Initialise the UART peripheral.
     * @param  config  UART configuration (baud, word length, parity, etc.).
     * @return Result::Ok on success.
     */
    Result init(const UartConfig& config = UartConfig{});

    /// De-initialise the UART peripheral and disable clock.
    Result deinit();

    /**
     * @brief  Blocking transmit.
     * @param  data       Pointer to data to send.
     * @param  size       Number of bytes to send.
     * @param  timeoutMs  Timeout per byte in milliseconds.
     */
    Result transmit(const uint8_t* data, uint16_t size, uint32_t timeoutMs = 1000);

    /**
     * @brief  Blocking receive (polled, not ring-buffer).
     * @param[out] data   Buffer for received data.
     * @param  size       Number of bytes to receive.
     * @param  timeoutMs  Timeout per byte in milliseconds.
     */
    Result receive(uint8_t* data, uint16_t size, uint32_t timeoutMs = 1000);

    /** @return Number of bytes available in the RX ring buffer. */
    uint16_t getRxAvailable() const;

    /**
     * @brief  Read bytes from the RX ring buffer.
     * @param[out] data   Buffer to fill.
     * @param  maxSize    Maximum number of bytes to read.
     * @return Number of bytes actually read.
     */
    uint16_t readRxBuffer(uint8_t* data, uint16_t maxSize);

    /** @brief  Enable or disable UART interrupts (RXNE, TC, PE). */
    void enableInterrupts(bool enable = true);

    /**
     * @brief  Callback called from ISR on each received byte.
     * @param  byte  The received byte.
     * @note   Called from ISR context — keep minimal.
     */
    void onDataReceived(uint8_t byte);

private:
    uint32_t instance_;        ///< UART instance number (1–8)
    bool initialized_;         ///< Whether the peripheral is configured
    UartConfig config_;        ///< Cached configuration

    // Ring buffer
    uint8_t* rxBuffer_;        ///< Dynamically allocated RX ring buffer
    uint16_t rxHead_;          ///< Producer index (written by ISR)
    uint16_t rxTail_;          ///< Consumer index (read by application)
    uint16_t rxBufferSize_;    ///< Size of the ring buffer

    /// @return Base address of the UART instance registers.
    volatile uint32_t* getBaseAddr() const;

    void enableClock();
    void configureGpio();

    /// Compute USART_BRR divider value (RM0433 §47.5.4).
    uint32_t computeBaudDiv(uint32_t baudrate) const;

    // USART register offsets
    static constexpr uint32_t USART_CR1_OFF = 0x00U;
    static constexpr uint32_t USART_CR2_OFF = 0x04U;
    static constexpr uint32_t USART_CR3_OFF = 0x08U;
    static constexpr uint32_t USART_BRR_OFF = 0x0CU;
    static constexpr uint32_t USART_RQR_OFF = 0x18U;
    static constexpr uint32_t USART_ISR_OFF = 0x1CU;
    static constexpr uint32_t USART_ICR_OFF = 0x20U;
    static constexpr uint32_t USART_RDR_OFF = 0x24U;
    static constexpr uint32_t USART_TDR_OFF = 0x28U;
};

/** @} */  // end of uart_driver group

} // namespace drone::drivers
