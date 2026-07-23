/**
 * @file    command_receiver.hpp
 * @brief   Command receiver — CRC-16 validated vehicle command parser.
 *
 * @details Provides a command receiver that accepts raw byte data and
 *          parses it into structured @ref core::VehicleCommand values:
 *          - CRC-16 validation with configurable polynomial (0x8005)
 *          - Packet format: opcode + optional payload + CRC16 (2 bytes)
 *          - Ring-buffer ingestion for stream data
 *          - Valid/invalid packet statistics
 *
 *          Supported commands: Arm, Disarm, Takeoff, Land, Hold, SetAttitude.
 *
 * @ingroup components
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "core/drone_types.hpp"

namespace drone::components {

/**
 * @defgroup command_receiver Command Receiver
 * @brief    CRC-16 validated vehicle command parser.
 *
 * ### Usage
 * @code{.cpp}
 *   CommandReceiver rx;
 *   uint8_t packet[] = {0x01, 0x00, 0x00};         // Command + CRC16
 *   rx.ingest(packet, 3);
 *   core::VehicleCommand cmd;
 *   if (rx.receive(cmd)) {
 *       // process cmd
 *   }
 * @endcode
 *
 * @{
 */

class CommandReceiver {
public:
    /** @brief  Attempt to parse the next valid command from the buffer.
     *  @param  command  [out] Parsed vehicle command.
     *  @return true if a valid command was parsed. */
    bool receive(core::VehicleCommand& command);

    /** @brief  Ingest raw data into the internal ring buffer.
     *  @param  data  Pointer to received bytes.
     *  @param  size  Number of bytes. */
    void ingest(const uint8_t* data, std::size_t size);

    /** @brief  Reset the receiver (clear buffer and statistics). */
    void reset();

    /// Compute CRC-16 for a data buffer.
    static uint16_t calculateCrc16(const uint8_t* data, std::size_t length);

    /// Validate a packet's CRC.
    bool validatePacket(const uint8_t* data, std::size_t size);

    /// @return Total valid packets received.
    uint32_t getValidPackets() const noexcept;

    /// @return Total invalid packets received.
    uint32_t getInvalidPackets() const noexcept;

private:
    uint8_t buffer_[128]{};           ///< Internal ring buffer
    std::size_t size_ = 0;            ///< Bytes currently in buffer
    uint32_t validPackets_ = 0;       ///< Valid packet counter
    uint32_t invalidPackets_ = 0;     ///< Invalid packet counter

    static constexpr uint16_t CRC16_POLY = 0x8005;

    bool parseCommand(const uint8_t* data, std::size_t size,
                      core::VehicleCommand& command);
};

/** @} */  // end of command_receiver group

} // namespace drone::components
