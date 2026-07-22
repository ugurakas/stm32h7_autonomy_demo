#pragma once

#include "components/autonomy_controller.hpp"
#include "drivers/system_clock.hpp"

namespace drone::app {

class Application {
public:
    Application() = default;
    void run();

private:
    components::AutonomyController controller_{};
};

} // namespace drone::app
