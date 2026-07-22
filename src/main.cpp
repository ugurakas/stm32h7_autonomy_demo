#include "app/drone_app.hpp"
#include "runtime/globals.hpp"

extern "C" int main(void) {
    drone::app::DroneApplication app;
    app.run();
    return 0;
}
