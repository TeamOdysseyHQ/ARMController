# Rovah Arm Controller Firmware

ESP32 firmware that reads seven AS5600 magnetic encoders through a TCA9548A I2C multiplexer and publishes raw counts and encoder angles to ROS 2 over micro-ROS serial.

## Hardware

- ESP32 Dev Module
- TCA9548A multiplexer at `0x70`
- Seven AS5600 encoders at `0x36`, each on a separate MUX channel
- SDA: GPIO 21 | SCL: GPIO 22 | Status LED: GPIO 2

## Joint mapping

Both topics use this array order:

| MUX channel | Joint |
| --- | --- |
| 0 | `base` |
| 1 | `joint_5` |
| 2 | `joint_3` |
| 3 | `joint_4` |
| 4 | `joint_2` |
| 5 | `trigger` |
| 6 | `joint_1` |

## ROS 2 topics

Node: `arm_encoder_node`. Configured publication target: 50 Hz.

| Topic | Message | Data |
| --- | --- | --- |
| `/arm_control/joint_raw` | `std_msgs/msg/Int32MultiArray` | Raw 12-bit counts, 0-4095 |
| `/arm_control/joint_states` | `sensor_msgs/msg/JointState` | Joint names and angles in degrees |

Angles use `raw_count * 360 / 4096`. For example, a count of `2048` gives `180` degrees.

This project's `JointState.position` uses degrees; standard ROS rotational joint positions use radians. The values are encoder angles without joint calibration offsets.

## Build and upload

Run from this directory with PlatformIO installed:

```sh
pio run -e esp32dev
pio run -e esp32dev -t upload
```

With a compatible micro-ROS serial agent running and ROS 2 sourced, inspect the topics:

```sh
ros2 topic echo /arm_control/joint_raw
ros2 topic echo /arm_control/joint_states
```

Run the topic commands in separate terminals. The LED blinks while waiting for the agent and stays on while connected.

## Standalone debugging

Set `USE_MICROROS` to `0` in [src/main.cpp](src/main.cpp), rebuild, and upload. Then open:

```sh
pio device monitor -b 115200
```

The serial table shows raw counts, degrees, magnet status, and count changes every 500 ms. Restore `USE_MICROROS` to `1` for ROS operation.

## Files

- `src/main.cpp`: active firmware
- `platformio.ini`: board and build configuration
- `notes/`: working notes and reference images
- `test/`: earlier sketches and hardware diagnostics

The firmware build passed; the angle update still needs hardware validation. Disconnected encoders retain their last reading, and sensor validity is not included in the ROS messages.
