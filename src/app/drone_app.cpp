#include "app/drone_app.hpp"
#include "drivers/system_clock.hpp"
#include "runtime/globals.hpp"

namespace drone::app {

void DroneApplication::init() {
    // Initialize system clock first
    SystemClock_Config();
    
    // Initialize status LED
    {
        drivers::GpioConfig cfg;
        cfg.mode = 1;
        cfg.pull = 0;
        cfg.speed = 2;
        cfg.outputType = 0;
        statusLed_.configure(cfg);
    }
    
    // Initialize button
    {
        drivers::GpioConfig cfg;
        cfg.mode = 0;  // Input
        cfg.pull = 1;  // Pull-up
        cfg.speed = 0;
        cfg.outputType = 0;
        button_.configure(cfg);
    }
    
    // Initialize subsystems
    adcDriver_.init();
    imuSensor_.init();
    motorDriver_.init();
    telemetryLink_.init();
    ledStatus_.init();
    
    // State initialization
    armed_ = false;
    autonomousMode_ = false;
    loopTime_ = 0.001f;  // 1kHz default
    loopCount_ = 0;
    lastTelemetryTime_ = 0;
    batteryVoltage_ = 12.6f;
    
    // Ready indication
    statusLed_.write(1);
    SystemClock::delayMs(100);
    statusLed_.write(0);
}

void DroneApplication::run() {
    init();
    
    uint32_t lastTime = SystemClock::getTickMs();
    
    while (true) {
        uint32_t now = SystemClock::getTickMs();
        float dt = (now - lastTime) * 0.001f;
        if (dt <= 0.0f) dt = 0.001f;
        if (dt > 0.01f) dt = 0.01f;  // Cap at 10ms
        lastTime = now;
        
        // Update sensors
        updateSensors();
        
        // Process commands
        core::VehicleCommand command{};
        if (commandReceiver_.receive(command)) {
            if (command.type == core::CommandType::Arm) {
                armed_ = flightController_.arm();
                if (!armed_) {
                    errorManager_.reportError(components::ErrorCode::ArmDenied);
                }
            } else if (command.type == core::CommandType::Disarm) {
                armed_ = !flightController_.disarm();
                if (!armed_) {
                    autonomousMode_ = false;
                }
            } else if (command.type == core::CommandType::SetAttitude && armed_) {
                flightController_.acceptCommand(command);
            } else if (command.type == core::CommandType::Takeoff && armed_) {
                flightController_.acceptCommand(command);
            } else if (command.type == core::CommandType::Land && armed_) {
                flightController_.acceptCommand(command);
            } else if (command.type == core::CommandType::Hold && armed_) {
                flightController_.acceptCommand(command);
                autonomousMode_ = true;
            }
        }
        
        // Check failsafe
        if (checkFailsafe()) {
            if (!errorManager_.hasError(components::ErrorCode::CommunicationLoss)) {
                errorManager_.reportError(components::ErrorCode::CommunicationLoss);
            }
            flightController_.triggerFailsafe();
            armed_ = false;
        } else {
            if (errorManager_.hasError(components::ErrorCode::CommunicationLoss)) {
                errorManager_.clearError(components::ErrorCode::CommunicationLoss);
                flightController_.clearFailsafe();
            }
        }
        
        // Update control
        updateControl(dt);
        
        // Update actuators
        updateActuators();
        
        // Update telemetry
        updateTelemetry();
        
        // Update status indicators
        updateStatus();
        
        // Handle user button
        handleButton();
        
        // Update error manager
        errorManager_.update(dt);
        
        // Error handling
        if (errorManager_.hasError()) {
            components::ErrorCode severe = errorManager_.getMostSevereError();
            if (severe == components::ErrorCode::BatteryLow) {
                flightController_.triggerFailsafe();
                armed_ = false;
            }
        }
        
        // Camera (periodic)
        if (loopCount_ % 100 == 0 && cameraStream_.isReady()) {
            core::CameraFrame frame{};
            cameraStream_.captureFrame(frame);
            (void)frame;
        }
        
        ++drone::runtime::counter;
        ++loopCount_;
        
        // Simple loop rate control
        uint32_t elapsed = SystemClock::getTickMs() - now;
        if (elapsed < 1) {
            // Busy wait for ~1kHz loop
            while (SystemClock::getTickMs() - now < 1) { }
        }
    }
}

void DroneApplication::updateSensors() {
    // Read IMU
    auto imuReading = imuSensor_.read();
    (void)imuReading;
    
    // Read battery voltage
    batteryVoltage_ = adcDriver_.readBatteryVoltage();
    
    // Check battery health
    if (batteryVoltage_ < 10.5f) {
        errorManager_.reportError(components::ErrorCode::BatteryLow);
    } else if (batteryVoltage_ > 11.0f) {
        if (errorManager_.hasError(components::ErrorCode::BatteryLow)) {
            errorManager_.clearError(components::ErrorCode::BatteryLow);
        }
    }
}

void DroneApplication::updateControl(float dt) {
    if (autonomousMode_ && armed_) {
        // Autonomous mode: use autonomy controller
        autonomyController_.update(dt);
        
        core::VehicleCommand autoCmd;
        autoCmd.type = core::CommandType::SetAttitude;
        autoCmd.roll = autonomyController_.state().roll;
        autoCmd.pitch = autonomyController_.state().pitch;
        autoCmd.yaw = autonomyController_.state().yaw;
        autoCmd.throttle = autonomyController_.state().throttle;
        flightController_.acceptCommand(autoCmd);
    }
    
    // Update flight controller
    flightController_.update(dt);
}

void DroneApplication::updateActuators() {
    const auto& state = flightController_.state();
    
    drivers::MixerInputs mixerInputs{};
    mixerInputs.thrust = state.throttle;
    mixerInputs.roll = state.roll;
    mixerInputs.pitch = state.pitch;
    mixerInputs.yaw = state.yaw;
    
    const auto motorOutput = drivers::MotorMixer::mix(mixerInputs);
    motorDriver_.setAllOutputs(
        motorOutput.m1, motorOutput.m2, 
        motorOutput.m3, motorOutput.m4
    );
}

void DroneApplication::updateTelemetry() {
    uint32_t now = SystemClock::getTickMs();
    
    // Send telemetry at 10Hz
    if (now - lastTelemetryTime_ >= 100) {
        lastTelemetryTime_ = now;
        
        core::TelemetryPacket telemetry{};
        telemetry.sequence = drone::runtime::counter;
        telemetry.state = flightController_.state();
        telemetry.batteryVoltage = batteryVoltage_;
        
        telemetryLink_.sendTelemetry(telemetry);
    }
}

void DroneApplication::updateStatus() {
    bool hasFault = errorManager_.hasError();
    bool batteryLow = errorManager_.hasError(components::ErrorCode::BatteryLow);
    
    ledStatus_.setSystemState(armed_, hasFault, autonomousMode_, batteryLow);
    ledStatus_.update(loopTime_);
}

void DroneApplication::handleButton() {
    static bool lastButtonState = true;
    static uint32_t lastDebounceTime = 0;
    
    bool currentState = button_.read();
    uint32_t now = SystemClock::getTickMs();
    
    if (currentState != lastButtonState && (now - lastDebounceTime) > 50) {
        lastDebounceTime = now;
        
        if (!currentState) {
            // Button pressed (pull-up, so 0 = pressed)
            if (armed_) {
                // Disarm
                armed_ = !flightController_.disarm();
                autonomousMode_ = false;
            } else {
                // Arm
                armed_ = flightController_.arm();
                if (!armed_) {
                    errorManager_.reportError(components::ErrorCode::ArmDenied);
                }
            }
        }
        
        lastButtonState = currentState;
    }
}

bool DroneApplication::checkFailsafe() {
    // Check if command receiver hasn't received valid packets
    static uint32_t lastValidPacket = 0;
    uint32_t now = SystemClock::getTickMs();
    
    uint32_t validPackets = commandReceiver_.getValidPackets();
    static uint32_t lastValidCount = 0;
    
    if (validPackets != lastValidCount) {
        lastValidPacket = now;
        lastValidCount = validPackets;
    }
    
    // Failsafe if no valid packets for 2 seconds
    return (now - lastValidPacket) > 2000 && loopCount_ > 100;
}

} // namespace drone::app
