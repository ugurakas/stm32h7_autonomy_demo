# Renode Simulation for STM32H7 Autonomy Demo

This folder contains the [Renode](https://renode.io) simulation files to run the drone firmware on a virtual STM32H753 board.

## Files

| File | Description |
|------|-------------|
| `stm32h753.repl` | Platform definition — thin wrapper around Renode's own maintained `platforms/cpus/stm32h753.repl` (CPU, NVIC, RCC, all peripherals) |
| `drone_simulation.resc` | Simulation script — loads the ELF, attaches a telemetry analyzer, sets ADC battery samples |

## Prerequisites

1. **Renode** v1.16+ — [Download](https://renode.io/downloads/)
2. **PlatformIO build** — `pio run -e nucleo_h753zi` (generates `.pio/build/nucleo_h753zi/firmware.elf`)

## Quick Start

```bash
# Build the firmware
pio run -e nucleo_h753zi

# Launch simulation (GUI)
renode renode/drone_simulation.resc
```

This opens a Monitor window and a telemetry terminal (USART3). The firmware
boots, configures its clock tree, and drives GPIOA5 (user LED) high then low
~100 ms later.

## Simulated Peripherals

| Peripheral | Address | Monitor name | Function |
|------------|---------|--------------|----------|
| USART3 | `0x40004800` | `usart3` | Telemetry output (115200 baud) — `TelemetryLink` uses `UartDriver` instance 3, i.e. USART3, **not** UART4 |
| TIM2 | `0x40000000` | `timer2` | PWM motor driver (4 channels) |
| I2C1 | `0x40005400` | `i2c1` | MPU6050 IMU bus (no stock IMU model shipped with Renode — reads back as NACK/zero) |
| ADC1 | `0x40022000` | `adcM1S2` | Battery voltage / VREFINT |
| GPIOA/B/C | `0x58020000`/`0x58020400`/`0x58020800` | `gpioPortA`/`B`/`C` | User LED (PA5), RGB status LEDs, user button (PC13) |

## Useful Monitor Commands

These are real Renode monitor commands, verified against this build:

```
# Attach a live terminal to telemetry output (GUI mode)
showAnalyzer usart3

# Read the CPU program counter (headless sanity check that it's running)
sysbus.cpu PC

# Read GPIOA's output data register (bit 5 = user LED state)
sysbus ReadDoubleWord 0x58020014

# Run a fixed slice of virtual time, then pause automatically
emulation RunFor "1s"

# Inject a battery/VREFINT ADC sample
adcM1S2 SetDefaultValue 2500 0   # channel 0, 2.5 V (=> 12.5 V through the divider)
adcM1S2 SetDefaultValue 1650 1   # channel 1, VREFINT
```

## Known Limitations

- **No arm/disarm/takeoff over UART in this build.** `CommandReceiver::ingest()`
  is never called anywhere in the firmware — no UART RX interrupt or other
  source feeds it bytes. Sending data to any UART in the simulation (or on
  real hardware) currently has no effect on vehicle state. This is a gap in
  the firmware's UART wiring, not a simulation limitation.
- **No IMU model.** Renode does not ship a stock MPU6050 peripheral, so I2C1
  has no simulated slave attached. IMU reads NACK/return zero. This only
  affects the (currently unused) autonomy IMU path.
- **PLL lock is not modeled.** Renode's stock RCC peripheral never sets
  `RCC_CR.PLL1RDY`, so `SystemClock::init()`'s PLL wait always times out in
  simulation (it would lock in ~100 µs on real silicon). The firmware falls
  back to treating the core as HSI-clocked (64 MHz) in that case, so SysTick
  and all millisecond timing still work correctly — just at a lower rate
  than the intended 400 MHz.

## Testing

The simulation is useful for:
- Verifying boot sequence and LED heartbeat
- Inspecting peripheral register state headlessly (GPIO, ADC, UART) via `sysbus ReadDoubleWord`
- Validating PID controller response (once command RX wiring exists)
- Testing telemetry frame construction over USART3
