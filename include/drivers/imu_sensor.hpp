#pragma once

namespace drone::drivers {

struct ImuReading {
    float accelX = 0.0f;
    float accelY = 0.0f;
    float accelZ = 0.0f;
    float gyroX = 0.0f;
    float gyroY = 0.0f;
    float gyroZ = 0.0f;
    float temperatureC = 25.0f;
};

class ImuSensor {
public:
    virtual ~ImuSensor() = default;
    virtual void init() = 0;
    virtual ImuReading read() = 0;
};

class MockImuSensor : public ImuSensor {
public:
    void init() override;
    ImuReading read() override;
};

} // namespace drone::drivers
