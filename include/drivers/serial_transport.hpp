#pragma once

#include <cstdint>

namespace drone::drivers {

class SerialTransport {
public:
    void init();
    void write(const std::uint8_t* data, std::size_t size);
    void read(std::uint8_t* data, std::size_t size);
};

} // namespace drone::drivers
