#pragma once

#include "core/drone_types.hpp"

namespace drone::components {

class CameraStream {
public:
    void captureFrame(core::CameraFrame& frame) const;
    bool isReady() const noexcept;
};

} // namespace drone::components
