# Notice and license

## esp2ros2 firmware

The sketch under `firmware/` is derived from **ROSArduinoBridge**:

- Authors: Patrick Goebel, James Nugen (and contributors)
- Original project: Pi Robot / HBRC, ArbotiX-inspired serial bridge
- License: **BSD 3-Clause** (see copyright header in `firmware/ROSArduinoBridge/ROSArduinoBridge.ino`)

Modifications for buddy / esp2ros2:

- ESP32 target with `ESP32_ENC_COUNTER` and [ESP32Encoder](https://github.com/madhephaestus/ESP32Encoder) library
- Default **L298N** pin map in `motor_driver.h`
- Documented for ROS 2 Jazzy **buddy** package (`diffdrive_arduino`)

## ESP32Encoder library (dependency, not vendored)

- Repository: https://github.com/madhephaestus/ESP32Encoder  
- Install separately via Arduino Library Manager or copy from `~/ros2_ws/ESP32Encoder`  
- Follow that project’s license when distributing the library

## buddy ROS 2 package

Robot description, Nav2, and launch files live in `~/ros2_ws/src/buddy` (separate package, MIT / see buddy `NOTICE.md`).
