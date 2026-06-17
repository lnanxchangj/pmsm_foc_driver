# PMSM FOC Driver (STM32)

This project is a high-performance Permanent Magnet Synchronous Motor (PMSM) Field Oriented Control (FOC) driver based on the STM32F407 microcontroller. It integrates the STMicroelectronics Motor Control SDK (MCSDK) with a CANopen communication stack (CANopenNode) implementing the CiA 402 drive profile.

## Architecture Overview

- **Hardware:** STM32F407 Series MCU.
- **Motor Control Layer:** 
  - Uses **ST MCSDK v6.4.2** for FOC algorithms.
  - Handles high-frequency tasks (current loop, PWM generation) in ADC interrupts.
  - Handles medium-frequency tasks (speed/position loops, state machine) via the MCSDK task scheduler.
- **Communication Layer:**
  - **CANopenNode:** Open-source CANopen stack.
  - **CiA 402 Implementation:** Custom bridge that maps CANopen Object Dictionary entries to the MCSDK high-level API.
  - Supports standard motion modes: Profile Position (PP), Profile Velocity (PV), Cyclic Synchronous Position (CSP), etc.

## Building and Running

1. **IDE:** Use **Keil MDK-ARM** to open the project file: `pmsm_foc_driver/MDK-ARM/pmsm_foc_driver.uvprojx`.
2. **Configuration:** Hardware peripherals can be reconfigured using **STM32CubeMX** (`pmsm_foc_driver.ioc`).
3. **Build:** Use the "Rebuild All" button in Keil MDK-ARM.
4. **Flash:** Use a ST-LINK or J-Link debugger to flash the compiled file to the target STM32F407 board.

## License
Open-source under the respective licenses of ST MCSDK and CANopenNode.
