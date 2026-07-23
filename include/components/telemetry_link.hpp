/**
 * @file    telemetry_link.hpp
 * @brief   Telemetry link — framed UART telemetry output at 10 Hz.
 *
 * @details Implements a telemetry system that sends vehicle state to a
 *          ground station via UART:
 *          - Framed protocol: start marker (0xFE) + length + msg_id + payload + CRC16
 *          - 10 Hz telemetry output rate
 *          - Packed payload: flight mode, attitude (roll/pitch/yaw),
 *            throttle, altitude, heartbeat, battery voltage
 *          - Hardware CRC-16 (CCITT) for frame integrity
 *          - Packet sequence numbering
 *          - UART3 on USART3 (configurable via constructor)
 *
 * @ingroup components
 */

#pragma once

#include <cstdint>
#include "core/drone_types.hpp"
#include "drivers/uart_driver.hpp"

namespace drone::components {

/**
 * @defgroup telemetry_link Telemetry Link
 * @brief    Framed telemetry output over UART at 10 Hz.
 *
 * ### Telemetry Frame Format
 * Byte | 0 | 1 | 2 | 3 | 4 | 5–34 | 35–38 | 39–40
 * -----|---|---|---|---|---|------|-------|-------
 * Field | 0xFE | Length | MsgID | Seq (16) | Mode | State floats (6×4) | Battery (4) | CRC16 (2)
 *
 * @{
 */

class TelemetryLink {
public:
    TelemetryLink();

    /** @brief  Initialise UART3 at 115200 baud. */
    void init();

    /** @brief  Send one telemetry packet (blocking).
     *  @param  packet  Telemetry data to transmit. */
    void sendTelemetry(const core::TelemetryPacket& packet);

    /// @return true if the UART link has been initialised.
    bool isConnected() const noexcept;

    /// @return Total packets sent.
    uint32_t getPacketsSent() const noexcept;

    /// @brief  Reset sequence and statistics.
    void reset();

private:
    drivers::UartDriver uart_;   ///< UART3 for telemetry
    uint32_t sequence_;          ///< Frame sequence counter
    uint32_t packetsSent_;       ///< Total frames sent
    bool initialized_;           ///< Link initialised flag

    static constexpr uint8_t MAVLINK_START   = 0xFE;   ///< Frame start marker
    static constexpr uint8_t TELEMETRY_MSG_ID = 0x01;   ///< Telemetry message type

    /// Compute 16-bit CRC-CCITT.
    uint16_t calculateCrc(const uint8_t* data, std::size_t length);

    /// Send a single byte over UART.
    void sendByte(uint8_t b);
};

/** @} */  // end of telemetry_link group

} // namespace drone::components
