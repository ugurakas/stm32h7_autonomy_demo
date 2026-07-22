#include "components/telemetry_link.hpp"
#include "drivers/system_clock.hpp"

#include <cstdint>
#include <cstring>

namespace drone::components {

TelemetryLink::TelemetryLink()
    : uart_(3), sequence_(0), packetsSent_(0), initialized_(false) {
}

void TelemetryLink::init() {
    drivers::UartConfig cfg;
    cfg.baudrate = 115200;
    cfg.wordLength = 8;
    cfg.stopBits = 1;
    cfg.parity = false;
    cfg.useRxDma = false;
    cfg.useTxDma = false;
    cfg.rxBufferSize = 256;
    
    if (uart_.init(cfg) == drivers::UartDriver::Result::Ok) {
        initialized_ = true;
    }
}

void TelemetryLink::sendTelemetry(const core::TelemetryPacket& packet) {
    if (!initialized_) return;
    
    // Build telemetry frame:
    // [0xFE][length][msg_id][seq][state data][battery 4 bytes][crc16 2 bytes]
    uint8_t frame[40];
    std::size_t idx = 0;
    
    frame[idx++] = MAVLINK_START;        // Start marker
    frame[idx++] = 0;                    // Length placeholder
    frame[idx++] = TELEMETRY_MSG_ID;     // Message ID
    frame[idx++] = (sequence_ >> 0) & 0xFF;  // Sequence low
    frame[idx++] = (sequence_ >> 8) & 0xFF;  // Sequence high
    
    // Pack vehicle state
    frame[idx++] = static_cast<uint8_t>(packet.state.mode);
    
    auto packFloat = [&](float val) {
        uint32_t raw;
        std::memcpy(&raw, &val, sizeof(raw));
        frame[idx++] = (raw >> 0) & 0xFF;
        frame[idx++] = (raw >> 8) & 0xFF;
        frame[idx++] = (raw >> 16) & 0xFF;
        frame[idx++] = (raw >> 24) & 0xFF;
    };
    
    packFloat(packet.state.roll);
    packFloat(packet.state.pitch);
    packFloat(packet.state.yaw);
    packFloat(packet.state.throttle);
    packFloat(packet.state.altitudeMeters);
    
    // Heartbeat
    frame[idx++] = (packet.state.heartbeat >> 0) & 0xFF;
    frame[idx++] = (packet.state.heartbeat >> 8) & 0xFF;
    frame[idx++] = (packet.state.heartbeat >> 16) & 0xFF;
    frame[idx++] = (packet.state.heartbeat >> 24) & 0xFF;
    
    // Battery voltage
    uint32_t battRaw;
    std::memcpy(&battRaw, &packet.batteryVoltage, sizeof(battRaw));
    frame[idx++] = (battRaw >> 0) & 0xFF;
    frame[idx++] = (battRaw >> 8) & 0xFF;
    frame[idx++] = (battRaw >> 16) & 0xFF;
    frame[idx++] = (battRaw >> 24) & 0xFF;
    
    // Set length (excludes start marker and CRC)
    frame[1] = idx - 3;
    
    // Calculate CRC over payload (from length field onwards)
    uint16_t crc = calculateCrc(&frame[1], idx - 1);
    frame[idx++] = (crc >> 0) & 0xFF;
    frame[idx++] = (crc >> 8) & 0xFF;
    
    // Send frame
    uart_.transmit(frame, idx, 100);
    
    sequence_++;
    packetsSent_++;
}

bool TelemetryLink::isConnected() const noexcept {
    return initialized_;
}

uint32_t TelemetryLink::getPacketsSent() const noexcept {
    return packetsSent_;
}

void TelemetryLink::reset() {
    sequence_ = 0;
    packetsSent_ = 0;
}

uint16_t TelemetryLink::calculateCrc(const uint8_t* data, std::size_t length) {
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0x8408;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFF;
}

void TelemetryLink::sendByte(uint8_t b) {
    uart_.transmit(&b, 1, 100);
}

} // namespace drone::components
