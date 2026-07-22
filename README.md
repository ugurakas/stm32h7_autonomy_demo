# STM32H7 Autonomy Demo

A professional-style embedded C++ firmware scaffold for STM32H7 targets with a layered architecture for autonomy-oriented applications.

## Architecture
- Core: shared data types and domain models
- Components: reusable control and behavior modules
- Drivers: hardware abstraction and board support
- Application: orchestration and runtime startup
- Runtime: global state and platform glue

## What is included
- PlatformIO project for STM32H743 Nucleo-style target
- Modular C++ application entry point
- Separated autonomy controller component
- Minimal system clock placeholder and runtime globals

## Build
Open the project folder in VS Code and run:

```bash
pio run
```

## Recommended next steps
- Add STM32 HAL or CubeMX-generated drivers under the drivers layer
- Introduce IMU, barometer, PID, and PWM components
- Add UART/I2C/SPI abstractions and logging
- Replace the placeholder clock init with a real STM32H7 clock configuration
