#pragma once

#include <cstdint>
#include "core/drone_types.hpp"
#include "drivers/uart_driver.hpp"

namespace drone::components {

class TelemetryLink {
public:
    TelemetryLink();
    void init();
    void sendTelemetry(const core::TelemetryPacket& packet);
    bool isConnected() const noexcept;
    uint32_t getPacketsSent() const noexcept;
    void reset();

private:
    drivers::UartDriver uart_;
    uint32_t sequence_;
    uint32_t packetsSent_;
    bool initialized_;
    
    static constexpr uint8_t MAVLINK_START = 0xFE;
    static constexpr uint8_t TELEMETRY_MSG_ID = 0x01;
    
    uint16_t calculateCrc(const uint8_t* data, std::size_t length);
    void sendByte(uint8_t b);
};

} // namespace drone::components
