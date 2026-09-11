# Notice and License

## Firmware in this repository

The sketch under `firmware/` is derived from **ROSArduinoBridge**:

- Original authors: Patrick Goebel, James Nugen, and contributors
- Upstream project: [ros_arduino_bridge](https://github.com/hbrobotics/ros_arduino_bridge) (Pi Robot / HBRC)
- License: **BSD 3-Clause** (see copyright header in `ROSArduinoBridge.ino`)

### Modifications in this tree

- ESP32 target with `ESP32_ENC_COUNTER` and the [ESP32Encoder](https://github.com/madhephaestus/ESP32Encoder) library
- L298N pin map and chassis mapping flags
- MPU9250 IMU driver over raw I2C (`imu_driver.*`)
- Serial extensions for combined encoder + IMU state (`f`, `g`, `i`)
- Documentation for ROS 2 Jazzy / `diffdrive_arduino` integration

## ESP32Encoder (dependency, not vendored)

- Repository: https://github.com/madhephaestus/ESP32Encoder  
- Install separately (Arduino Library Manager or local copy)  
- Follow that project’s license when redistributing the library

## Host ROS 2 stack

Robot description, navigation, and launch files live in a separate repository:  
[buddy-ros2](https://github.com/mirzasalem/buddy-ros2).
