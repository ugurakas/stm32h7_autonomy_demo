#pragma once

#include <cstdint>

namespace drone::drivers {

struct MotorOutput {
    float frontLeft = 0.0f;
    float frontRight = 0.0f;
    float rearLeft = 0.0f;
    float rearRight = 0.0f;
};

class PwmMotorDriver {
public:
    virtual ~PwmMotorDriver() = default;
    virtual void init() = 0;
    virtual void setOutputs(const MotorOutput& output) = 0;
};

class MockPwmMotorDriver : public PwmMotorDriver {
public:
    void init() override;
    void setOutputs(const MotorOutput& output) override;
    const MotorOutput& lastOutput() const noexcept;

private:
    MotorOutput lastOutput_{};
};

class Stm32PwmMotorDriver : public PwmMotorDriver {
public:
    explicit Stm32PwmMotorDriver(std::uint32_t timerChannel = 1);
    void init() override;
    void setOutputs(const MotorOutput& output) override;

private:
    std::uint32_t timerChannel_;
};

} // namespace drone::drivers
