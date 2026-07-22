#pragma once

#include <cstdint>

namespace drone::core {

enum class FlightMode : std::uint8_t {
    Idle,
    Armed,
    Takeoff,
    Hover,
    Landing,
    Fault
};

enum class CommandType : std::uint8_t {
    None,
    Arm,
    Disarm,
    Takeoff,
    Land,
    Hold,
    SetAttitude
};

struct VehicleCommand {
    CommandType type = CommandType::None;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float throttle = 0.0f;
};

struct VehicleState {
    FlightMode mode = FlightMode::Idle;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float throttle = 0.0f;
    float altitudeMeters = 0.0f;
    std::uint32_t heartbeat = 0;
};

struct TelemetryPacket {
    std::uint32_t sequence = 0;
    VehicleState state{};
    float batteryVoltage = 12.0f;
};

struct CameraFrame {
    std::uint16_t width = 0;
    std::uint16_t height = 0;
    std::uint16_t bytes = 0;
    std::uint8_t payload[64] = {};
    bool valid = false;
};

} // namespace drone::core
