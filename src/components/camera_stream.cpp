#include "components/camera_stream.hpp"

namespace drone::components {

void CameraStream::captureFrame(core::CameraFrame& frame) const {
    frame.width = 640;
    frame.height = 480;
    frame.bytes = 64;
    frame.valid = true;
    for (std::size_t i = 0; i < 64; ++i) {
        frame.payload[i] = static_cast<std::uint8_t>(i % 251);
    }
}

bool CameraStream::isReady() const noexcept {
    return true;
}

} // namespace drone::components
