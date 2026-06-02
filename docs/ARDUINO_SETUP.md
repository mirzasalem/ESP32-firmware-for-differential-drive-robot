# Arduino IDE setup and upload

## 1. Install Arduino IDE

Use [Arduino IDE 2.x](https://www.arduino.cc/en/software) on Linux, Windows, or macOS.

## 2. ESP32 board support

1. **File → Preferences → Additional boards manager URLs** add:
   ```
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
2. **Tools → Board → Boards Manager** → search **esp32** → install **esp32** by Espressif.
3. **Tools → Board** → **esp32** → **ESP32 Dev Module** (or your exact module).

Recommended **Tools** settings:

| Setting | Value |
|---------|--------|
| Upload speed | 921600 (or 115200 if upload fails) |
| USB CDC On Boot | Enabled (if your board uses native USB) |
| Flash size | Match your module (often 4 MB) |

## 3. Install ESP32Encoder library

See [../libraries/README.md](../libraries/README.md).

```bash
cp -r ~/ros2_ws/ESP32Encoder ~/Arduino/libraries/ESP32Encoder
```

Library Manager version **0.11.7** is fine if already installed.

## 4. Open the sketch

**File → Open** → select:

```
~/esp/esp2ros2/firmware/ROSArduinoBridge/ROSArduinoBridge.ino
```

All tabs must appear: `motor_driver.ino`, `encoder_driver.ino`, `diff_controller.h`, etc.  
If you only see one tab, you opened the wrong folder.

## 5. Verify configuration

In `ROSArduinoBridge.ino` near the top:

```cpp
#define L298_MOTOR_DRIVER
#define ESP32_ENC_COUNTER
//#define CYTRON_MDD3A
```

Only **one** motor driver define should be active.

### Buddy-specific firmware options (already in this tree)

| File | What to know |
|------|----------------|
| `buddy_robot_config.h` | **`BUDDY_L298_ENCODER_CROSS 0`**, **`BUDDY_L298_MOTOR_CROSS 1`** (split cross — see WIRING.md) |
| `motor_driver.ino` | `setMotorSpeeds()` crosses motor PWM when `MOTOR_CROSS=1` |
| `encoder_driver.h` | LEFT encoder GPIO **26, 27**; RIGHT **16, 17** |
| `encoder_driver.ino` | `readEncoder(LEFT)` negated; **`readEncoderRosLeft/Right()`** use `ENCODER_CROSS` |
| `ROSArduinoBridge.ino` | `o` / `e` use ROS joint order |

Details: [WIRING.md](WIRING.md), [SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md).

## 6. Select port and upload

1. Plug ESP32 via USB.
2. **Tools → Port** → `/dev/ttyUSB1` or `/dev/ttyACM0`.
3. **Upload** (→).
4. Wait for **Done uploading**.

### Linux permissions

```bash
sudo usermod -aG dialout $USER
# log out and back in
```

On the robot, prefer buddy’s udev script: `buddy/scripts/setup_usb_serial.sh`.

## 7. Serial monitor test

**Tools → Serial Monitor** → **115200 baud** → line ending **CR** or **Both NL & CR**.

| Type + Enter | Expected |
|--------------|----------|
| `b` | `115200` |
| `e` | two integers (left count, right count) |
| `o 100 100` | both wheels move (same direction) |
| `o 100 -100` | spin in place (if wiring matches buddy) |
| `o 0 0` | stop |

Better: `~/esp/esp2ros2/scripts/test_motors_diag.sh /dev/ttyUSB1`

Close Serial Monitor before running buddy.

## 8. Run buddy

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch buddy robot_mapping.launch.py
# second terminal:
ros2 run buddy keyboard_teleop
```

Teleop: `i` forward, `,` back, `j` / `l` turn. See buddy [KEYBOARD_TELEOP.md](https://github.com/mirzasalem/buddy/blob/main/docs/KEYBOARD_TELEOP.md).

## Re-flash when to change what

| You changed | Action |
|-------------|--------|
| `motor_driver.*`, `encoder_driver.*`, `ROSArduinoBridge.ino` | **Upload** again |
| Buddy URDF / `controller.yaml` / `ros2_control.xacro` only | `colcon build` on Pi — no ESP reflash |
| Serial protocol or cross flags in `buddy_robot_config.h` | Re-flash + note in buddy `DRIVE_TRAIN.md` |

## PlatformIO (optional)

Not included by default. Arduino IDE matches buddy documentation.
