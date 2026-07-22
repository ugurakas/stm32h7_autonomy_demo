// STM32H7 Autonomy Demo - Unit Tests
// Compile with: g++ -std=c++11 -I../include -I. test_main.cpp ../src/components/pid_controller.cpp ../src/components/flight_controller.cpp ../src/components/command_receiver.cpp ../src/components/error_manager.cpp ../src/components/autonomy_controller.cpp ../src/components/camera_stream.cpp ../src/drivers/motor_mixer.cpp -o test_drone

#include "test_framework.hpp"
#include "core/drone_types.hpp"

// Include headers under test
#include "components/pid_controller.hpp"
#include "components/flight_controller.hpp"
#include "components/command_receiver.hpp"
#include "components/error_manager.hpp"
#include "components/autonomy_controller.hpp"
#include "components/camera_stream.hpp"
#include "drivers/motor_mixer.hpp"

#include <cmath>

using namespace drone::components;
using namespace drone::drivers;
using namespace drone::core;

// ============================================================================
// PID Controller Tests
// ============================================================================

void test_PidControllerBasics() {
    test::g_currentTest = "PidControllerBasics";
    
    // Use kd=0 to avoid derivative spike on step inputs
    PidController pid(PidController::Gains(1.0f, 0.1f, 0.0f));
    
    // Test zero error
    float output = pid.update(0.0f, 0.01f);
    ASSERT_NEAR(output, 0.0f, 0.001f, "Zero error should produce near-zero output");
    
    // Test proportional response (reset first to clear integral)
    pid.reset();
    float error = 1.0f;
    output = pid.update(error, 0.01f);
    ASSERT_GT(output, 0.0f, "Positive error should produce positive output");
    
    // Test negative error
    pid.reset();
    output = pid.update(-1.0f, 0.01f);
    ASSERT_TRUE(output < 0.0f, "Negative error should produce negative output");
    
    // Test reset clears state
    pid.reset();
    output = pid.update(0.0f, 0.01f);
    ASSERT_NEAR(output, 0.0f, 0.001f, "After reset, output should be near zero");
    
    printf("    [PASS] PID Basics: output=%.4f\n", output);
}

void test_PidControllerAntiWindup() {
    test::g_currentTest = "PidControllerAntiWindup";
    
    PidController pid(PidController::Gains(2.0f, 10.0f, 0.0f));
    
    // Set tight output limits
    pid.setOutputLimits(-0.5f, 0.5f);
    pid.setIntegralLimits(-0.1f, 0.1f);
    
    // Large sustained error should not cause unbounded integral windup
    float output = 0.0f;
    for (int i = 0; i < 1000; i++) {
        output = pid.update(1.0f, 0.01f);
    }
    
    // Output should be clamped
    ASSERT_TRUE(output <= 0.5f, "Output should be clamped to max limit");
    ASSERT_TRUE(output >= -0.5f, "Output should be clamped within min limit");
    
    // Integral should not have wound up
    ASSERT_TRUE(pid.integral() <= 0.5f, "Integral should be bounded by anti-windup");
    
    printf("    [PASS] PID Anti-windup: output=%.4f, integral=%.6f\n", output, pid.integral());
}

// ============================================================================
// Flight Controller Tests
// ============================================================================

void test_FlightControllerArmDisarm() {
    test::g_currentTest = "FlightControllerArmDisarm";
    
    FlightController fc;
    
    // Initially should not be armed
    ASSERT_EQ(fc.state().mode, FlightMode::Idle, "Initial mode should be Idle");
    ASSERT_TRUE(!fc.isArmed(), "Should not be armed initially");
    
    // Use arm() method
    ASSERT_TRUE(fc.arm(), "arm() should return true when disarmed");
    ASSERT_TRUE(fc.isArmed(), "isArmed() should be true after arm()");
    ASSERT_EQ(fc.state().mode, FlightMode::Armed, "Mode should be Armed");
    
    // Disarm
    ASSERT_TRUE(fc.disarm(), "disarm() should return true when armed");
    ASSERT_TRUE(!fc.isArmed(), "isArmed() should be false after disarm()");
    ASSERT_EQ(fc.state().mode, FlightMode::Idle, "Mode should be Idle after Disarm");
    ASSERT_NEAR(fc.state().throttle, 0.0f, 0.001f, "Throttle should be zero after disarm");
    
    printf("    [PASS] Arm/Disarm\n");
}

void test_FlightControllerStateMachine() {
    test::g_currentTest = "FlightControllerStateMachine";
    
    FlightController fc;
    VehicleCommand cmd;
    
    // Arm: use arm() method to transition internal state
    ASSERT_TRUE(fc.arm(), "arm() should succeed");
    ASSERT_EQ(fc.state().mode, FlightMode::Armed, "Mode should be Armed");
    
    // Takeoff via acceptCommand + update
    cmd.type = CommandType::Takeoff;
    fc.acceptCommand(cmd);
    fc.update(3.0f);  // 3 seconds: altitude = 0 + 0.5*3 = 1.5m
    bool modeIsHover = (fc.state().mode == FlightMode::Hover);
    ASSERT_TRUE(modeIsHover, "Mode should transition to Hover after reaching altitude");
    ASSERT_NEAR(fc.state().altitudeMeters, 1.5f, 0.1f, "Altitude should be near 1.5m");
    
    // Reset for Land test - start at some altitude
    fc.reset();
    fc.arm();
    cmd.type = CommandType::Takeoff;
    fc.acceptCommand(cmd);
    fc.update(1.0f);  // climb to 0.5m
    
    cmd.type = CommandType::Land;
    fc.acceptCommand(cmd);
    fc.update(0.01f);  // small descent: 0.5 - 0.3*0.01 = 0.497m, still > 0
    ASSERT_EQ(fc.state().mode, FlightMode::Landing, "Mode should be Landing");
    
    printf("    [PASS] State Machine: mode=%d alt=%.3fm\n", (int)fc.state().mode, fc.state().altitudeMeters);
}

// ============================================================================
// Command Receiver Tests
// ============================================================================

void test_CommandReceiverCRC() {
    test::g_currentTest = "CommandReceiverCRC";
    
    // Test CRC calculation
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t crc = CommandReceiver::calculateCrc16(data, 4);
    
    // CRC should be consistent
    uint16_t crc2 = CommandReceiver::calculateCrc16(data, 4);
    ASSERT_EQ(crc, crc2, "CRC should be deterministic");
    
    // Different data should produce different CRC
    uint8_t data2[] = {0x01, 0x02, 0x03, 0x05};
    uint16_t crc3 = CommandReceiver::calculateCrc16(data2, 4);
    ASSERT_TRUE(crc != crc3, "Different data should produce different CRC");
    
    printf("    [PASS] CRC-16: 0x%04X == 0x%04X\n", crc, crc2);
}

void test_CommandReceiverPacket() {
    test::g_currentTest = "CommandReceiverPacket";
    
    CommandReceiver receiver;
    
    // Test Arm command (opcode + 2-byte CRC)
    uint8_t armPacket[] = {0x01, 0x00, 0x00};  // Arm opcode, CRC placeholder (little-endian)
    uint16_t crc = CommandReceiver::calculateCrc16(armPacket, 1);
    armPacket[1] = (crc >> 0) & 0xFF;   // CRC low byte
    armPacket[2] = (crc >> 8) & 0xFF;   // CRC high byte
    
    receiver.ingest(armPacket, 3);
    
    VehicleCommand cmd;
    ASSERT_TRUE(receiver.receive(cmd), "Arm command with valid CRC should be received");
    ASSERT_EQ(cmd.type, CommandType::Arm, "Command type should be Arm");
    
    // Valid packet count should be > 0 after successful receive
    ASSERT_TRUE(receiver.getValidPackets() > 0, "Valid packet count should be > 0");
    
    printf("    [PASS] Packet: valid=%u, invalid=%u\n", receiver.getValidPackets(), receiver.getInvalidPackets());
}

// ============================================================================
// Motor Mixer Tests
// ============================================================================

void test_MotorMixer() {
    test::g_currentTest = "MotorMixer";
    
    // Test hover (all zero, thrust=0.5)
    MixerInputs hover;
    hover.thrust = 0.5f;
    MotorOutput out = MotorMixer::mix(hover);
    
    ASSERT_NEAR(out.frontLeft, 0.5f, 0.001f, "Hover: FL should be 0.5");
    ASSERT_NEAR(out.frontRight, 0.5f, 0.001f, "Hover: FR should be 0.5");
    ASSERT_NEAR(out.rearLeft, 0.5f, 0.001f, "Hover: RL should be 0.5");
    ASSERT_NEAR(out.rearRight, 0.5f, 0.001f, "Hover: RR should be 0.5");
    
    // Test roll left (negative roll)
    // MotorMixer formula: FL += roll, FR -= roll, RL -= roll, RR += roll
    // With roll=-0.5: FL-=0.35*0.5, FR+=0.35*0.5, RL+=0.35*0.5, RR-=0.35*0.5
    MixerInputs rollLeft;
    rollLeft.thrust = 0.5f;
    rollLeft.roll = -0.5f;
    out = MotorMixer::mix(rollLeft);
    ASSERT_TRUE(out.frontLeft < out.frontRight, "Roll left: FL < FR");
    // RL gets +roll (since -(-0.5) = +0.5), RR gets +roll (since +(-0.5) = -0.5)
    // So RL > RR
    ASSERT_TRUE(out.rearLeft > out.rearRight, "Roll left: RL > RR (rear opposite sign)");
    
    // Test pitch forward (positive pitch)
    MixerInputs pitchFwd;
    pitchFwd.thrust = 0.5f;
    pitchFwd.pitch = 0.5f;
    out = MotorMixer::mix(pitchFwd);
    ASSERT_TRUE(out.frontLeft > out.rearLeft, "Pitch forward: FL > RL");
    ASSERT_TRUE(out.frontRight > out.rearRight, "Pitch forward: FR > RR");
    
    // Test output clamping
    MixerInputs maxInput;
    maxInput.thrust = 0.5f;
    maxInput.roll = 2.0f;
    maxInput.pitch = 2.0f;
    maxInput.yaw = 2.0f;
    out = MotorMixer::mix(maxInput);
    ASSERT_TRUE(out.frontLeft <= 1.0f, "Output should be clamped to 1.0");
    ASSERT_TRUE(out.frontLeft >= 0.0f, "Output should be clamped to 0.0");
    
    // Test yaw (positive yaw = CW)
    // MotorMixer: FL -= yaw, FR += yaw, RL -= yaw, RR += yaw
    // With yaw=1.0: FL -= 0.25, FR += 0.25, RL -= 0.25, RR += 0.25
    MixerInputs yawInput;
    yawInput.thrust = 0.5f;
    yawInput.yaw = 1.0f;
    out = MotorMixer::mix(yawInput);
    ASSERT_TRUE(out.frontLeft < out.frontRight, "Yaw CW: FL < FR (FL decreases, FR increases)");
    
    printf("    [PASS] Motor Mixer\n");
}

// ============================================================================
// Error Manager Tests
// ============================================================================

void test_ErrorManager() {
    test::g_currentTest = "ErrorManager";
    
    ErrorManager em;
    
    // Should start healthy
    ASSERT_TRUE(em.isSystemHealthy(), "System should start healthy");
    ASSERT_TRUE(!em.hasError(), "No errors initially");
    
    // Report an error
    em.reportError(ErrorCode::ImuFailure);
    ASSERT_TRUE(em.hasError(), "Should have error after reporting");
    ASSERT_TRUE(!em.isSystemHealthy(), "System should be unhealthy with error");
    ASSERT_TRUE(em.hasError(ErrorCode::ImuFailure), "Should detect IMU failure");
    
    // Clear specific error
    em.clearError(ErrorCode::ImuFailure);
    ASSERT_TRUE(!em.hasError(ErrorCode::ImuFailure), "IMU error should be cleared");
    
    // Multiple errors
    em.reportError(ErrorCode::BatteryLow);
    em.reportError(ErrorCode::CommunicationLoss);
    ASSERT_EQ(em.getErrorCount(), 2, "Should track multiple errors");
    
    // Clear all
    em.clearAll();
    ASSERT_TRUE(!em.hasError(), "Should have no errors after clearAll");
    
    printf("    [PASS] Error Manager\n");
}

// ============================================================================
// Autonomy Controller Tests
// ============================================================================

void test_AutonomyController() {
    test::g_currentTest = "AutonomyController";
    
    AutonomyController ac;
    
    // Initial state should be near zero
    FlightState state = ac.state();
    ASSERT_NEAR(state.throttle, 0.0f, 0.01f, "Initial throttle should be near zero");
    
    // Reset
    ac.reset();
    state = ac.state();
    ASSERT_NEAR(state.throttle, 0.0f, 0.01f, "After reset, throttle should be near zero");
    
    // Update should produce non-zero values
    ac.update(0.01f);
    state = ac.state();
    ASSERT_TRUE(state.throttle > 0.5f, "Throttle should be > 0.5 after update");
    
    // Set waypoint
    ac.setMissionWaypoint(2.0f, 0.0f);
    ASSERT_NEAR(ac.getTargetAltitude(), 2.0f, 0.01f, "Target altitude should be 2.0m");
    
    printf("    [PASS] Autonomy: throttle=%.3f, target_alt=%.1fm\n", state.throttle, ac.getTargetAltitude());
}

// ============================================================================
// Camera Stream Tests
// ============================================================================

void test_CameraStream() {
    test::g_currentTest = "CameraStream";
    
    CameraStream camera;
    
    // Should always be ready
    ASSERT_TRUE(camera.isReady(), "Camera should be ready");
    
    // Capture a frame
    CameraFrame frame;
    camera.captureFrame(frame);
    
    ASSERT_TRUE(frame.valid, "Frame should be valid");
    ASSERT_EQ(frame.width, 640, "Width should be 640");
    ASSERT_EQ(frame.height, 480, "Height should be 480");
    ASSERT_EQ(frame.bytes, 64, "Bytes should be 64");
    
    // Payload should contain data
    bool nonZero = false;
    for (int i = 0; i < 64; i++) {
        if (frame.payload[i] != 0) {
            nonZero = true;
            break;
        }
    }
    ASSERT_TRUE(nonZero, "Payload should contain non-zero data");
    
    printf("    [PASS] Camera: %ux%u, %u bytes, valid=%d\n", frame.width, frame.height, frame.bytes, frame.valid);
}

// ============================================================================
// Main
// ============================================================================

int main() {
    printf("\n========================================\n");
    printf("  STM32H7 AUTONOMY DEMO - UNIT TESTS\n");
    printf("========================================\n\n");

    // Run tests
    printf("--- PID Controller ---\n");
    test_PidControllerBasics();
    test_PidControllerAntiWindup();
    
    printf("\n--- Flight Controller ---\n");
    test_FlightControllerArmDisarm();
    test_FlightControllerStateMachine();
    
    printf("\n--- Command Receiver ---\n");
    test_CommandReceiverCRC();
    test_CommandReceiverPacket();
    
    printf("\n--- Motor Mixer ---\n");
    test_MotorMixer();
    
    printf("\n--- Error Manager ---\n");
    test_ErrorManager();
    
    printf("\n--- Autonomy Controller ---\n");
    test_AutonomyController();
    
    printf("\n--- Camera Stream ---\n");
    test_CameraStream();
    
    // Print results
    printResults();
    
    return (test::failedCount == 0) ? 0 : 1;
}
