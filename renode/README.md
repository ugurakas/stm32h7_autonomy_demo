# Renode Simulation for STM32H7 Autonomy Demo

This folder contains the [Renode](https://renode.io) simulation files to run the drone firmware on a virtual STM32H753 board.

## Files

| File | Description |
|------|-------------|
| `stm32h753.repl` | Platform definition (peripherals, memory map, interrupts) |
| `drone_simulation.resc` | Simulation script — loads ELF, sets up UART, I2C sensors, GPIO monitors |

## Prerequisites

1. **Renode** v1.14+ — [Download](https://renode.io/downloads/)
2. **PlatformIO build** — `pio run` (generates `.pio/build/nucleo_h743zi/firmware.elf`)

## Quick Start

```bash
# Build the firmware
pio run

# Launch simulation
renode renode/drone_simulation.resc

# In Renode's monitor, connect to UART4 terminal:
#   (machine-1) uart4_terminal
```

## Simulated Peripherals

| Peripheral | Address | Function |
|------------|---------|----------|
| UART4 | `0x40004C00` | Debug console + telemetry output (115200 baud) |
| TIM2 | `0x40000000` | PWM motor driver (4 channels) |
| I2C1 | `0x40012000` | MPU6050 IMU sensor bus |
| ADC1 | `0x40022000` | Battery voltage monitoring |
| GPIOA | `0x58020000` | User LED (PA5), motor PWM AF |
| GPIOC | `0x58020800` | User button (PC13, pull-up) |
| GPIOB | `0x58020400` | RGB status LEDs (PB0, PB7, PB14) |

## Commands

From the Renode monitor:

```
# Connect to UART4 terminal
(machine-1) uart4_terminal

# Send a command (arm the drone)
(machine-1) "$elf" send "arm"

# Toggle the user button (PC13)
(machine-1) gpioc TogglePin 13

# Check GPIO states
(machine-1) gpioa PeekPin 5

# Read ADC channels
(machine-1) adc1 ReadValue 0
```

## Testing

The simulation is ideal for:
- Verifying boot sequence and LED heartbeat
- Testing arm/disarm state machine
- Validating PID controller response
- Simulating sensor failure and recovery
- Testing telemetry output format

