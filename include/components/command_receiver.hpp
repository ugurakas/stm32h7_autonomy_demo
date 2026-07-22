#pragma once

#include <cstddef>
#include <cstdint>

#include "core/drone_types.hpp"

namespace drone::components {

class CommandReceiver {
public:
    bool receive(core::VehicleCommand& command);
    void ingest(const uint8_t* data, std::size_t size);
    void reset();
    
    // CRC validation
    static uint16_t calculateCrc16(const uint8_t* data, std::size_t length);
    bool validatePacket(const uint8_t* data, std::size_t size);
    
    // Stats
    uint32_t getValidPackets() const noexcept;
    uint32_t getInvalidPackets() const noexcept;

private:
    uint8_t buffer_[128]{};
    std::size_t size_ = 0;
    uint32_t validPackets_ = 0;
    uint32_t invalidPackets_ = 0;
    
    static constexpr uint16_t CRC16_POLY = 0x8005;
    
    bool parseCommand(const uint8_t* data, std::size_t size, core::VehicleCommand& command);
};

} // namespace drone::components
