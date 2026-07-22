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
