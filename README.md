# BalancingRobot-Main-Controller

Main controller firmware of a two-wheeled self-balancing robot (engineering thesis project).
Runs on an **STM32F411CE (Blackpill)**: reads the MPU6050 IMU, runs the control cascade at 200 Hz,
commands the ESP32 motor driver over UART and streams telemetry to the PC over USB.

Part of the Balancing Robot project:

| Repository | Role |
| --- | --- |
| **BalancingRobot-Main-Controller** (this repo) | STM32 firmware: IMU, control cascade, state machine, telemetry |
| [BalancingRobot-SimpleFOC-Motor-Controller](https://github.com/GrzegorzGrudzinski/BalancingRobot-SimpleFOC-Motor-Controller) | ESP32 + SimpleFOC motor driver (current control of both BLDC motors) |
| [BalancingRobot-simulatior](https://github.com/GrzegorzGrudzinski/BalancingRobot-simulatior) | PyBullet simulator, runs this repo's core through `sil/` |
| [BalancingRobot-DataAnalyser](https://github.com/GrzegorzGrudzinski/BalancingRobot-DataAnalyser) | PC telemetry parser, monitor and logger |

## Hardware

| Part | Connection |
| --- | --- |
| STM32F411CE Blackpill | main MCU |
| MPU6050 IMU | I2C2 (PB3 SDA, PB10 SCL) |
| ESP32 on MKS DualFOC v3.2 plus | USART2, 460800 baud |
| Start / clear-fault button | PA0 (active low) |
| PC | USB CDC (virtual COM port) |

Pin and peripheral mapping: `code/Core/Inc/App/platform/hardware_config.h`, CubeMX project:
`code/BalancingRobot-Main-Controller.ioc`.

## How it works

- **Control loop:** TIM3 interrupt every 5 ms (200 Hz, `LOOP_TIME` in `robot_config.h`).
- **Controller:** cascade PID: position → velocity → pitch → motor current, plus a yaw loop
  (`CONTROLLER` in `robot_config.h`). The output is a current command [A] for each wheel.
- **State machine:** `STANDBY → CONNECT → INIT → BALANCING`, with `FALLEN`, `ERROR` and `STOP`.
  The button starts the handshake with the ESP32; the robot starts balancing once it is held
  upright (pitch within `START_ANGLE` for `UPRIGHT_TIME`).
- **PC commands** (USB, frame start `0xDD`): `HEARTBEAT`, `STOP`, `SET_PARAM` (live gain tuning),
  `SET_TARGET` (position / velocity / yaw targets). After the PC takes over, no valid frame for
  300 ms stops the robot (`PC_TIMEOUT`).
- **Telemetry** (USB, frame start `0xCC`, versioned): every control cycle, decoded by
  `telemetry_frame.py` in BalancingRobot-DataAnalyser.

Diagrams (draw.io / PNG) are in `resources/`.

## Repository layout

```
code/                      STM32CubeMX + CMake project
  Core/Inc|Src/App/
    core/                  HAL-free robot core (also built on the PC in sil/)
      config/              robot parameters and defaults (RobotParams_t)
      estimation/          wheel odometry
      supervisor/          robot state machine, targets, robotCoreStep()
      protocol/            STM <-> ESP frames
      telemetry/           telemetry frame packing
      diag/                diagnostic counters
    control/               controllers (PID, cascade PID); self-contained, swappable;
                           glue to the robot only in controllers_config.c
    communication/         UART to the ESP32, USB commands and telemetry (HAL side)
    IO/                    MPU6050 + Kalman filter, button
    platform/              hardware mapping, DWT cycle counter
    config/config.h        temporary include (robot_config.h + hardware_config.h)
    robot_app.c            main application: ties the HAL to the core
sil/                       PC build of the core: unit tests and a shared library for Python
resources/                 diagrams
```

## Building and flashing

Requirements: CMake ≥ 3.22, Ninja and the `arm-none-eabi` GCC toolchain (for example the one
installed by the STM32Cube VS Code extension).

```bash
cd code
cmake --preset Debug          # or Release
cmake --build build/Debug     # -> build/Debug/BalancingRobot-Main-Controller.elf
```

Flashing and debugging: ST-Link via the STM32Cube VS Code extension
(`.vscode/launch.json`, "STM32Cube: Launch ST-Link GDB Server"), or any ST-Link tool with the `.elf`.

## Software-in-the-loop (`sil/`)

The core and the controllers are built for the PC with the host compiler, without HAL:

```bash
make -C sil check   # compile only: proves the core needs no HAL
make -C sil test    # unit tests (test_core.c)
make -C sil lib     # libcontrol_core.so for Python
```

Python wrappers (ctypes): `sil/control_core.py` (cascade only) and `sil/robot_core.py` (whole core:
odometry, state machine, PC commands, telemetry). The simulator uses them to run exactly the same C
code as the robot.

The pitch angle is an input of the core: on the robot it comes from the Kalman filter in
`IO/mpu6050.c`, in SiL the caller provides it.

> **Note:** the SiL layer (`sil/`, wrappers `control_core.py`, `robot_core.py`) is a temporary
> prototype written with AI assistance to try out the approach; it will be rewritten.

## Configuration

- `code/Core/Inc/App/core/config/robot_config.h`: geometry, loop time, timeouts, limits, default gains
- `code/Core/Inc/App/platform/hardware_config.h`: peripherals and pins
- `code/Core/Inc/App/config/config.h`: includes both of the above (temporary)

The defaults are loaded into `RobotParams_t` (`core/config/robot_params.c`); gains can also be
changed at run time from the PC with `SET_PARAM`.
