#pragma once

#include "components/autonomy_controller.hpp"
#include "components/camera_stream.hpp"
#include "components/command_receiver.hpp"
#include "components/error_manager.hpp"
#include "components/flight_controller.hpp"
#include "components/led_status.hpp"
#include "components/telemetry_link.hpp"
#include "drivers/adc_driver.hpp"
#include "drivers/gpio_driver.hpp"
#include "drivers/mpu6050_driver.hpp"
#include "drivers/motor_mixer.hpp"
#include "drivers/pwm_driver.hpp"
#include "drivers/system_clock.hpp"

namespace drone::app {

class DroneApplication {
public:
    void init();
    void run();

private:
    // Core flight
    components::FlightController flightController_{};
    components::AutonomyController autonomyController_{};
    
    // Communication
    components::CommandReceiver commandReceiver_{};
    components::TelemetryLink telemetryLink_{};
    
    // Sensors
    drivers::Mpu6050Driver imuSensor_{};
    drivers::AdcDriver adcDriver_{1};  // ADC1 for battery
    
    // Actuators
    drivers::PwmDriver motorDriver_{2, 1};  // TIM2 channels 1-4
    
    // Status & safety
    components::LedStatus ledStatus_{};
    components::ErrorManager errorManager_{};
    components::CameraStream cameraStream_{};
    
    // GPIO
    drivers::GpioDriver statusLed_{drivers::GpioPort::PortA, 5};
    drivers::GpioDriver button_{drivers::GpioPort::PortC, 13};
    
    // Internal state
    bool armed_;
    bool autonomousMode_;
    float loopTime_;
    uint32_t loopCount_;
    uint32_t lastTelemetryTime_;
    float batteryVoltage_;
    
    void updateSensors();
    void updateControl(float dt);
    void updateActuators();
    void updateTelemetry();
    void updateStatus();
    void handleButton();
    bool checkFailsafe();
};

} // namespace drone::app
