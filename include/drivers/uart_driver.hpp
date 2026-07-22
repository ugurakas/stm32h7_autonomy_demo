#pragma once

#include <cstdint>

namespace drone::drivers {

struct UartConfig {
    uint32_t baudrate = 115200UL;
    uint32_t wordLength = 8;        // 7 or 8 bits
    uint32_t stopBits = 1;          // 1 or 2 stop bits
    bool parity = false;
    bool parityEven = false;
    bool useRxDma = false;
    bool useTxDma = false;
    uint16_t rxBufferSize = 256;
};

class UartDriver {
public:
    enum class Result {
        Ok,
        ErrorBusy,
        ErrorTimeout,
        ErrorOverrun,
        ErrorFraming,
        ErrorParity,
        ErrorInvalidParam
    };

    UartDriver(uint32_t instance = 1);  // USART1-8
    ~UartDriver();

    Result init(const UartConfig& config = UartConfig{});
    Result deinit();
    
    Result transmit(const uint8_t* data, uint16_t size, uint32_t timeoutMs = 1000);
    Result receive(uint8_t* data, uint16_t size, uint32_t timeoutMs = 1000);
    
    uint16_t getRxAvailable() const;
    uint16_t readRxBuffer(uint8_t* data, uint16_t maxSize);
    
    void enableInterrupts(bool enable = true);
    
    // Callback for received data (called from ISR context)
    void onDataReceived(uint8_t byte);

private:
    uint32_t instance_;
    bool initialized_;
    UartConfig config_;
    
    // Ring buffer for RX
    uint8_t* rxBuffer_;
    uint16_t rxHead_;
    uint16_t rxTail_;
    uint16_t rxBufferSize_;
    
    volatile uint32_t* getBaseAddr() const;
    void enableClock();
    void configureGpio();
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

} // namespace drone::drivers
