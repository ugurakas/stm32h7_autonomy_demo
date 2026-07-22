# STM32H7 Autonomy Demo
This project focuses on building a modular embedded software structure for future autonomous vehicle applications.
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

## Overview

The goal of this project is to create a scalable firmware architecture for autonomous systems by separating hardware drivers, application logic, and control algorithms.

The current implementation includes:

- Hardware abstraction layer
- Sensor drivers
- Flight control modules
- PID based control structure
- Telemetry communication
- Modular C++ software architecture
- Basic autonomy framework

## System Architecture

The software is organized into multiple layers:

## Current Features

### Embedded Software

- C++ based firmware structure
- Modular component design
- Hardware independent driver layer
- Expandable architecture for different MCU platforms

### Control System

Implemented modules:

- PID controller framework
- Flight state management
- Motor mixing structure
- Sensor abstraction layer

### Hardware Interfaces

Supported interfaces:

- UART
- I2C
- PWM
- ADC
- GPIO

## Target Hardware

The project is designed around STM32H7 series microcontrollers.

Target platform:

- ARM Cortex-M7
- High performance real-time embedded applications
- Autonomous systems development

## Development Environment

Recommended tools:

- VS Code
- PlatformIO
- ARM GCC Toolchain

## Roadmap

### Phase 1 - Software Foundation

[x] Project architecture  
[x] Driver abstraction  
[x] Component based structure  
[x] Basic control modules  


### Phase 2 - Hardware Integration

[ ] STM32H753 target board  
[ ] IMU integration  
[ ] Barometer integration  
[ ] GPS interface  
[ ] CAN communication  


### Phase 3 - Autonomous Platform

[ ] Sensor fusion  
[ ] Navigation algorithms  
[ ] Real-time task management  
[ ] Mission planning framework  


## Motivation

This project is part of a long-term effort to develop embedded software and hardware platforms for autonomous systems.

The main focus is learning and implementing the core technologies behind modern autonomous vehicles:

- Embedded systems
- Real-time control
- Sensor integration
- Robotics software architecture

## License

This project is currently under development.
