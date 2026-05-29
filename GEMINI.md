# Project: PMSM FOC Driver (STM32)

This project is a high-performance Permanent Magnet Synchronous Motor (PMSM) Field Oriented Control (FOC) driver based on the STM32F407 microcontroller. It integrates the STMicroelectronics Motor Control SDK (MCSDK) with a CANopen communication stack (CANopenNode) implementing the CiA 402 drive profile.

## Architecture Overview

- **Hardware:** STM32F407 Series MCU.
- **Motor Control Layer:** 
  - Uses **ST MCSDK v6.4.2** for FOC algorithms.
  - Handles high-frequency tasks (current loop, PWM generation) in ADC interrupts.
  - Handles medium-frequency tasks (speed/position loops, state machine) via the MCSDK task scheduler.
- **Communication Layer:**
  - **CANopenNode:** Open-source CANopen stack.
  - **CiA 402 Implementation:** Custom bridge (`cia402.c`) that maps CANopen Object Dictionary entries to the MCSDK high-level API (`mc_api.h`).
  - Supports standard motion modes: Profile Position (PP), Profile Velocity (PV), Cyclic Synchronous Position (CSP/CSV/CST), etc.
- **Application Layer:**
  - `main.c`: Orchestrates hardware initialization, Motor Control setup, and the CANopen process loop.

## Key Components

- `pmsm_foc_driver/Src/main.c`: Application entry point and main process loop.
- `pmsm_foc_driver/CANopen/CANopenNode_STM32/cia402.c`: Implementation of the CiA 402 state machine and motion control modes.
- `pmsm_foc_driver/Src/mc_tasks.c`: Motor control task definitions and initialization.
- `pmsm_foc_driver/Inc/mc_api.h`: High-level interface for motor control commands (Start, Stop, Set Speed, etc.).
- `pmsm_foc_driver/pmsm_foc_driver.ioc`: STM32CubeMX configuration file.
- `tools/`: Python-based testing suite for CANopen and CiA 402 verification.

## Building and Running

### Firmware (C/C++)
1. **IDE:** Use **Keil MDK-ARM** to open the project file: `pmsm_foc_driver/MDK-ARM/pmsm_foc_driver.uvprojx`.
2. **Configuration:** Hardware peripherals can be reconfigured using **STM32CubeMX** (`pmsm_foc_driver.ioc`).
3. **Build:** Use the "Rebuild All" button in Keil.
4. **Flash:** Use a ST-LINK or J-Link debugger to flash the `.axf`/`.hex` file to the target STM32F407 board.

### Testing Tools (Python)
The `tools/` directory contains scripts for automated testing over the CAN bus.
1. **Environment:** Python 3.x is required.
2. **Dependencies:** Install required packages via `pip install -r tools/requirements.txt`.
3. **Run Tests:**
   - `python tools/cia402_test.py`: Verifies CiA 402 state transitions and motion commands.
   - `python tools/auto_test.py`: Runs a suite of automated motor tests.

## Development Conventions

- **Coding Standard:** Follows STMicroelectronics MCSDK and HAL coding conventions (primarily MISRA C:2012 compliant where applicable).
- **Communication:** CANopen Node ID is defaulted to `0x01` (1) and baudrate to `500 Kbps` (configurable in `main.c`).
- **Motor Control:** Always use the `mc_api.h` functions to interact with the motor to ensure the internal state machine remains synchronized.
- **Safety:** The `cia402.c` implementation includes fault reaction states. Ensure hardware-level protections (over-current, over-voltage) are correctly configured in MCSDK.

## Documentation References
- [STM32 Motor Control SDK Documentation](https://www.st.com/en/embedded-software/x-cube-mcsdk.html)
- [CANopenNode GitHub Repository](https://github.com/CANopenNode/CANopenNode)
- [CiA 402 - CANopen device profile for drives and motion control](https://www.can-cia.org/groups/specifications/)
