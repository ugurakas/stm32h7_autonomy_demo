/**
 * @file    drone_types.hpp
 * @brief   Core domain types — shared data structures.
 *
 * @details Defines the fundamental data types used across all layers:
 *          - @ref FlightMode (Idle, Armed, Takeoff, Hover, Landing, Fault)
 *          - @ref CommandType (Arm, Disarm, Takeoff, Land, Hold, SetAttitude)
 *          - @ref VehicleCommand — structured input from RC/ground station
 *          - @ref VehicleState — current estimated state of the vehicle
 *          - @ref TelemetryPacket — telemetry frame data
 *          - @ref CameraFrame — placeholder camera frame structure
 *
 *          These types are shared between drivers, components, and the
 *          application layer to ensure type-safe communication.
 *
 * @ingroup core
 */

#pragma once

#include <cstdint>

namespace drone::core {

/**
 * @defgroup core Core Domain Types
 * @brief    Shared data structures for the drone application.
 * @{
 */

/** @brief  High-level flight mode of the vehicle. */
enum class FlightMode : std::uint8_t {
    Idle,          ///< System idle, motors disarmed
    Armed,         ///< Motors armed, ready for flight
    Takeoff,       ///< Automatic takeoff sequence
    Hover,         ///< Altitude hold at target
    Landing,       ///< Automatic landing sequence
    Fault          ///< Failsafe / error state
};

/** @brief  Command types accepted from a remote controller or GCS. */
enum class CommandType : std::uint8_t {
    None,          ///< No command
    Arm,           ///< Arm motors
    Disarm,        ///< Disarm motors
    Takeoff,       ///< Initiate takeoff
    Land,          ///< Initiate landing
    Hold,          ///< Hold position/altitude
    SetAttitude    ///< Set attitude target (roll, pitch, yaw, throttle)
};

/** @brief  Structured vehicle command from RC or autonomy. */
struct VehicleCommand {
    CommandType type = CommandType::None;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float throttle = 0.0f;
};

/** @brief  Current estimated state of the vehicle. */
struct VehicleState {
    FlightMode mode = FlightMode::Idle;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float throttle = 0.0f;
    float altitudeMeters = 0.0f;
    std::uint32_t heartbeat = 0;
};

/** @brief  Telemetry frame transmitted to the ground station. */
struct TelemetryPacket {
    std::uint32_t sequence = 0;
    VehicleState state{};
    float batteryVoltage = 12.0f;
};

/** @brief  Camera frame placeholder (mock for testing). */
struct CameraFrame {
    std::uint16_t width = 0;
    std::uint16_t height = 0;
    std::uint16_t bytes = 0;
    std::uint8_t payload[64] = {};
    bool valid = false;
};

/** @} */  // end of core group

} // namespace drone::core
