# Update Log

This file lists the bug fixes and missing-piece completions made to the
project, in date order. Each entry: file, problem, and fix applied.

Verification: `pio run -e nucleo_h743zi` and `pio run -e nucleo_h753zi` both
build clean; all 55 host-side unit tests in `test/build_and_run_tests.bat`
pass.

---

## 2026-08-01

### RCC (clock/register) offset bugs — severe enough that the system would never boot

Register offsets were verified against the official STMicroelectronics
CMSIS header (`stm32h753xx.h`, `RCC_TypeDef`). Almost every RCC register
address except `CR`, `D1CFGR`/`D2CFGR`/`D3CFGR`, and `AHB4ENR` pointed at a
different register — or a reserved region — on real silicon.

- **`src/drivers/system_clock.cpp`**
  - `RCC_CFGR` `0x08` → `0x10`, `RCC_PLLCKSELR` `0x0C` → `0x28`,
    `RCC_PLLCFGR` `0x10` → `0x2C`, `RCC_PLL1DIVR` `0x14` → `0x30`,
    `RCC_BDCR` `0xA0` → `0x70` (this address also collided with `APB2ENR`
    in `uart_driver.cpp`/`pwm_driver.cpp`).
  - `PWR_D3CR` address `0x58024808` → `0x58024818` (the real
    `PWR_TypeDef.D3CR` offset is `0x18`, not `0x08`).
  - `configurePll()`: the `PLLSRC` field belongs in `PLLCKSELR` bits[1:0]
    but was mistakenly written to `PLLCFGR` bit 0; `DIVM1` was written to
    bits[3:0] instead of bits[9:4]; `DIVP1EN`/`DIVQ1EN`/`DIVR1EN` (bits
    16/17/18) were never set at all — meaning even a locked PLL produced
    no P/Q/R output; the `PLL1DIVR` R field was written to bit 25 instead
    of bit 24.
  - `init()`: `CFGR.SWS` (clock-switch-status) lives at bits[5:3], but the
    code read bits[3:2] — even after switching to PLL1 this check could
    never confirm success, wasting ~1,000,000 busy-wait iterations to
    timeout on every boot.
  - `configureLse()`: `LSERDY` is bit 1, but the code checked bit 2.
  - The bus prescaler fields (`D1CFGR`/`D2CFGR`/`D3CFGR`) were at the
    wrong bit positions; the code also wrote `D1CPRE=/2`, which
    contradicted the 400 MHz CPU target the function reports at the end
    (D1CPRE=/2 would leave the CPU at 200 MHz). Fixed to `D1CPRE=/1`.
  - The `PWR_D3CR` VOS field was handled at bits[1:0] instead of
    bits[15:14]; the "ready" check should poll the `VOSRDY` flag (bit 13),
    not the selection bits — fixed.

- **`src/drivers/uart_driver.cpp`**: `RCC_APB1LENR` `0x60`→`0xE8`,
  `RCC_APB2ENR` `0xA0`→`0xF0` (addresses were wrong; bit positions were
  already correct).

- **`src/drivers/pwm_driver.cpp`**: same `APB1LENR`/`APB2ENR` address fix.

- **`src/drivers/i2c_driver.cpp`**: `APB1LENR` address fixed; `APB4ENR`
  address `0xE4` (actually `APB3ENR`) → `0xF4` fixed; the I2C1/2/3 enable
  bit formula `20 + instance - 1` shifted every instance one bit too low
  (I2C1 landed on bit 20 instead of the real bit 21) → fixed to
  `20 + instance`; the I2C4 enable bit was moved from bit 4 to its real
  position, bit 7.

### `src/drivers/adc_driver.cpp` — ADC never actually worked

- `ADC123_COMMON` was defined as `0x40022100`, which is actually
  `ADC2_BASE` itself — VBAT/VSENSE selection was writing into ADC2's own
  `IER` register. Fixed to the real common block, `ADC1_BASE + 0x300`
  (`0x40022300`).
- The ADC1/2/3 clock-enable bits were defined as `RCC_AHB4ENR` bit5/bit6 —
  those bits are actually the GPIOF/GPIOG clock enables (they collide
  exactly with `GpioPort::PortF/PortG` in `gpio_driver.cpp`). Real
  location: `RCC_AHB1ENR` bit 5 for ADC1/2, `RCC_AHB4ENR` bit 24 for ADC3.
  Fixed; as a result the ADC never received a clock and the `ADRDY`
  wait loop in `init()` would spin forever.

### `src/drivers/pwm_driver.cpp` — motor channels 2 and 4 produced no PWM at all

Only the low byte of `CCMR1`/`CCMR2` (channel 1 / channel 3) was being
configured; the high byte (channel 2 / channel 4) stayed at its reset
"Frozen" mode value. Since `setAllOutputs()` drives all four motors,
motors 2 and 4 never received a PWM signal. Fixed so both the low and
high byte of each register are written with the same bit layout.

### `src/components/pid_controller.cpp` — derivative term had the wrong sign

The code computed `output = pTerm + iTerm - dTerm`; the standard PID form
is `output = pTerm + iTerm + dTerm`. The inverted sign made the
derivative term amplify oscillation instead of damping it (the flight
controller's roll/pitch/yaw PIDs all use this). Fixed.

### `src/components/error_manager.cpp` — `clearError` underflowed on a double call

`clearError()` decremented `errorCount_` without checking whether the
record was actually `active`; calling it twice in a row (or on an
already-inactive error) underflowed the unsigned `errorCount_` to
~4 billion, which made `hasError()`/`isSystemHealthy()` permanently
report a fault. The existing `if (errorCount_ < 0)` guard could never
trigger either, since `errorCount_` is unsigned (dead code). Added an
`record->active` check.

### `src/drivers/imu_sensor.cpp` — mock IMU had shared static state

`MockImuSensor::read()` used a function-local `static float angle`; if
more than one `MockImuSensor` were constructed, all instances would share
the same angle. `angle_` is now a proper class member
(`include/drivers/imu_sensor.hpp`).

### `src/drivers/stm32_pwm_timer.cpp` + `pwm_motor_driver.cpp` — placeholder, produced no PWM

`Stm32PwmTimer::init()`/`setDutyCycle()` were completely empty stubs
(comment: "Placeholder for real STM32 timer configuration"). Since
`Stm32PwmMotorDriver` used this, motors on real hardware never received
any signal at all. `Stm32PwmTimer` now wraps the already-correct
register-level `PwmDriver`. Also, `Stm32PwmMotorDriver::setOutputs()` was
constructing a brand-new `Stm32PwmTimer` and calling `init()` on every
single call (re-running hardware init at control-loop rate); `timer_` is
now a member, initialised once.

### `test/build_and_run_tests.bat` — test suite never actually compiled

`test_framework.cpp` and `mock_system_clock.cpp` were missing from the
`SOURCES` list; the script failed with linker errors on `test::testCount`
and friends. Both were added. All 55 tests now pass.

---

## Fixes already applied and verified before this session

The following were already present, uncommitted, in the repo before this
review; they were checked during this session and confirmed correct
(no further changes made):

- `include/drivers/i2c_driver.hpp`: I2C1/2/3 base addresses corrected
  (`0x40012000`/`13000`/`14000` → the real `0x40005400`/`5800`/`5C00`).
- `src/components/command_receiver.cpp`: a packet that fails CRC
  validation is no longer executed as a command; CRC byte order
  (little-endian) fixed; replaced a strict-aliasing-violating
  `reinterpret_cast` with `memcpy`.
- `src/components/flight_controller.cpp`: `altVelocity` moved from a
  function-local `static` to a class member (shared-state bug across
  multiple instances).
- `src/drivers/mpu6050_driver.cpp`: replaced taking the address of a
  `constexpr` member (an odr-use violation) with a local copy.
- `src/drivers/system_clock.cpp`: SysTick is now configured even if
  HSE/PLL bring-up fails (otherwise the failsafe timer would never run
  at all); added the `HSI_VALUE` constant.
- `src/app/drone_app.cpp`: fixed missing `drivers::` namespace qualifiers
  and `MotorMixer` output field names (`m1..m4` → `frontLeft` etc.).
- `src/system_stm32h7xx.c`: removed a placeholder definition that
  conflicted with the real `SystemClock_Config` (multiple-definition /
  link-error risk).
