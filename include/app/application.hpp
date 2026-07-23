/**
 * @file    application.hpp
 * @brief   Legacy application entry — superseded by DroneApplication.
 *
 * @details Simple modular application class that runs the autonomy
 *          controller in a minimal loop.  This class is kept for
 *          backward compatibility; all new development should use
 *          @ref drone::app::DroneApplication instead.
 *
 * @ingroup app
 */

#pragma once

#include "components/autonomy_controller.hpp"
#include "drivers/system_clock.hpp"

namespace drone::app {

/**
 * @defgroup app Application Layer
 * @brief    Top-level application orchestration.
 * @{
 */

/**
 * @brief  Minimal application — loops the autonomy controller.
 *
 * @deprecated Use @ref DroneApplication for full flight stack.
 */
class Application {
public:
    Application() = default;
    void run();

private:
    components::AutonomyController controller_{};
};

/** @} */

} // namespace drone::app
