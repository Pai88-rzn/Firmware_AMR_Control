# Firmware_AMR_Control

Firmware for the General Purpose AMR Platform (GPRP), including the STM32 bridge controller and dedicated STM32 IMU controller.

## Projects

- `STM32_Bridge/`: STM32F411 bridge firmware for safety I/O, sensors, relays, USB, FreeRTOS, and micro-ROS integration.
- `STM32_IMU/`: STM32F103 IMU firmware for BNO055 acquisition and telemetry.
- `tools/`: Host-side simulation and diagnostic utilities.

See [agents.md](agents.md) for the hardware architecture, pin mapping, communication protocols, and development rules.

## Build STM32_IMU

```powershell
cmake -B STM32_IMU/build -S STM32_IMU -G Ninja --toolchain STM32_IMU/arm-none-eabi-toolchain.cmake
cmake --build STM32_IMU/build
```

The ARM GCC toolchain, CMake, and Ninja must be installed and available on `PATH`.
