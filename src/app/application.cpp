/**
 * @file    application.cpp
 * @brief   Legacy application — simple autonomy loop.
 *
 * @details Initialises the system clock then runs the autonomy
 *          controller at ~1 kHz.  This is a minimal placeholder
 *          superseded by @ref drone::app::DroneApplication.
 *
 * @ingroup app
 */

#include "app/application.hpp"
#include "runtime/globals.hpp"

#include <cstdint>

namespace drone::app {

void Application::run() {
    SystemClock_Config();

    while (true) {
        controller_.update(0.001f);
        ++drone::runtime::counter;
    }
}

} // namespace drone::app
