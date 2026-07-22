#include "drivers/serial_transport.hpp"

namespace drone::drivers {

// Initialize the serial link used for commands and telemetry.
void SerialTransport::init() {
}

// Send a byte buffer to the communication link.
void SerialTransport::write(const std::uint8_t* data, std::size_t size) {
    (void)data;
    (void)size;
}

// Read incoming bytes from the communication link into a buffer.
void SerialTransport::read(std::uint8_t* data, std::size_t size) {
    (void)data;
    (void)size;
}

} // namespace drone::drivers
