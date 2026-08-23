# Arduino IDE setup and upload

Buddy ships with firmware tuned and **verified on the real robot** — upload this tree before first drive, run the closed-loop bench test, then launch buddy navigation.

## Documentation map

| Doc | Contents |
|-----|----------|
| [../README.md](../README.md) | Overview, **firmware source table**, IMU integration, run commands |
| [WIRING.md](WIRING.md) | Power, L298, encoders, MPU9250 I2C pins |
| [SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md) | All serial commands (`m`, `e`, `f`, `g`, `i`, `u`) |

## Firmware source (what each tab does)

Open **`~/esp/esp2ros2/firmware/ROSArduinoBridge/ROSArduinoBridge.ino`**. All tabs must appear.

| Tab / file | Purpose |
|------------|---------|
| `ROSArduinoBridge.ino` | Serial command loop, `#define USE_IMU`, dispatches to handlers |
| `commands.h` | Command letter → handler IDs |
| `buddy_robot_config.h` | `ENCODER_CROSS`, `MOTOR_CROSS`, `LEFT_ENCODER_INVERT` |
| `motor_driver.ino` | L298 PWM on GPIO 18/19/32/33 |
| `encoder_driver.ino` | ESP32Encoder counts; ROS left/right read functions |
| `diff_controller.h` | 50 Hz velocity PID — Ki ramps PWM under load |
| `imu_driver.ino` | MPU9250 I2C read, bias calibration, `imuReadSerial()` for `f`/`g` |
| `imu_driver.h` | SDA 21, SCL 22, 100 Hz, milli-unit serial format |

Re-flash after editing any of these. Pi-only YAML/xacro changes do not require reflash unless serial protocol changes.

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
#define USE_IMU
```

Only **one** motor driver define should be active.

`USE_IMU` enables the MPU9250 on I2C (SDA 21 / SCL 22). It needs no extra library — `imu_driver.ino` talks to the registers directly through `Wire`, which also means it works with the MPU6500 boards sold as MPU9250. Comment it out if no IMU is fitted; the sketch then behaves exactly as before and `f` reports `imu_ok 0`.

### Buddy-specific firmware options (already in this tree)

| File | What to know |
|------|----------------|
| `buddy_robot_config.h` | **`BUDDY_L298_ENCODER_CROSS 0`**, **`BUDDY_L298_MOTOR_CROSS 1`**, **`BUDDY_LEFT_ENCODER_INVERT 1`** (required — left encoder sign for PID + `e`) |
| `motor_driver.ino` | `setMotorSpeeds()` crosses motor PWM when `MOTOR_CROSS=1` |
| `diff_controller.h` | **Velocity PID** (ROS1 joey_v1 port): PWM from 0, **Ki** ramps under load — no PWM-130 floor |
| `encoder_driver.h` | LEFT encoder GPIO **26, 27**; RIGHT **16, 17** |
| `encoder_driver.ino` | `readEncoder(LEFT)` negated; **`readEncoderRosLeft/Right()`** use `ENCODER_CROSS` |
| `ROSArduinoBridge.ino` | `o` / `e` use ROS joint order |
| `imu_driver.h` / `.ino` | MPU9250 raw I2C, ±250 °/s, 100 Hz, gyro bias averaged at boot. Magnetometer unused; mount orientation belongs in buddy `imu.xacro` |

Details: [WIRING.md](WIRING.md), [SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md).

### IMU wiring (when `USE_IMU` is on)

| MPU9250 | ESP32 |
|---------|-------|
| VCC | **3.3 V** (not 5 V on logic pins) |
| GND | GND (common with L298 / battery) |
| SDA | GPIO **21** |
| SCL | GPIO **22** |

Robot must be **still at USB power-on** so `imuInit()` can average gyro bias (200 samples).

## 6. Select port and upload

1. Plug ESP32 via USB.
2. **Tools → Port** → `/dev/ttyACM0`, `/dev/ttyACM1`, or `/dev/ttyUSB1`.
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
| `u 100:40:150:50` | `OK` — buddy default PID (P:D:I:Ko) |
| `r` | `OK` — reset encoders + PID |
| `e` | two integers (left count, right count) |
| `m 5 5` | wheels move — **no reply** (fire-and-forget) |
| `m 0 0` | stop |
| `o 100 100` | both wheels move → `OK 100 100` |
| `o 100 -100` | spin in place (if wiring matches buddy) |
| `g` | `gx gy gz ax ay az 1` — gyro near 0 while still, `az` near 9800 (mm/s²). `…0` means no IMU on the bus; `Invalid Command` means `USE_IMU` is off |
| `i` | `OK` — re-estimate gyro bias (keep the robot still) |
| `o 0 0` | stop |

**Closed-loop bench (recommended after upload):**

```bash
~/esp/esp2ros2/scripts/test_closed_loop.sh /dev/ttyACM0 3
```

**Open-loop wiring check:**

```bash
~/esp/esp2ros2/scripts/test_motors_diag.sh /dev/ttyACM0
```

Close Serial Monitor before running buddy or the bench scripts.

## 7b. IMU bench pass (required before `use_imu:=true`)

With ROS **stopped** and the robot **still**:

```bash
~/esp/esp2ros2/scripts/test_closed_loop.sh /dev/ttyACM0 2
```

Or in Serial Monitor, type **`g`** + Enter. Expected pass:

```text
g -> 0 -1 -1  151 131 9761  1
         │              │    └── imu_ok: must be 1
         │              └── az ≈ 9800 mm/s² (gravity, board flat)
         └── gyro ≈ 0 mrad/s while still
```

| Result | Action |
|--------|--------|
| Last field **`1`** | IMU OK — proceed to buddy launch with `use_imu:=true` |
| Last field **`0`** | No device on I2C — check wiring ([WIRING.md](WIRING.md)) |
| `Invalid Command` | Re-flash with `#define USE_IMU` enabled |
| Gyro large while still | Send **`i`** (bias recalibrate) or power-cycle while still |

Test combined read (what the Pi uses):

```bash
python3 -c "import serial,time; s=serial.Serial('/dev/ttyACM0',115200,timeout=1); time.sleep(2); s.write(b'f\r'); time.sleep(0.2); print(s.read(128).decode())"
```

Reply format: `left right gx gy gz ax ay az imu_ok`

## 8. Run buddy (drive-only default)

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch buddy robot_mapping.launch.py
# second terminal:
ros2 run buddy keyboard_teleop
```

Teleop: `i` forward, `,` back, `j` / `l` turn. See buddy [KEYBOARD_TELEOP.md](https://github.com/mirzasalem/buddy/blob/main/docs/KEYBOARD_TELEOP.md).

## 9. Run buddy with MPU9250 fusion

**Prerequisites:** IMU bench pass (§7b), Pi packages installed:

```bash
sudo apt install ros-jazzy-robot-localization ros-jazzy-imu-sensor-broadcaster
colcon build --packages-select diffdrive_arduino buddy --symlink-install
source ~/ros2_ws/install/setup.bash
```

**Run this:**

```bash
ros2 launch buddy robot_navigation.launch.py \
  map:=/home/mirza/ros2_ws/src/buddy/maps/my_map.yaml \
  use_imu:=true
```

| Step | Action |
|------|--------|
| 1 | Wait **~20 s** for controllers, EKF, AMCL, Nav2 |
| 2 | Second terminal: `ros2 run buddy keyboard_teleop` |
| 3 | Press **`k`**, then **`i`** — RViz model drives **forward** in facing direction |
| 4 | Optional: **2D Pose Estimate**, send Nav2 goal |

**Verify:**

```bash
ros2 control list_controllers   # imu_sensor_broadcaster + diff_drive_controller active
ros2 topic hz /imu_sensor_broadcaster/imu
ros2 topic hz /odom
```

What changes on the Pi when `use_imu:=true`:

| | `use_imu:=false` | `use_imu:=true` |
|--|--|--|
| ESP poll | `e` | **`f`** (encoders + IMU) |
| Extra ROS nodes | `odom_relay` | `imu_sensor_broadcaster`, `ekf_filter_node` |
| `/odom` | relay from controller | EKF fused (wheel vx + gyro vyaw) |

Firmware behaviour is the same either way — only the Pi launch arg changes which serial command is polled.

Full integration doc: [../README.md#mpu9250-imu-optional](../README.md#mpu9250-imu-optional) and buddy [DRIVE_TRAIN.md §9](https://github.com/mirzasalem/buddy-ros2/blob/main/docs/DRIVE_TRAIN.md#9-imu-and-odometry-fusion--mpu9250-optional).

## 10. Re-flash when to change what

| You changed | Action |
|-------------|--------|
| `motor_driver.*`, `encoder_driver.*`, `diff_controller.h`, `imu_driver.*`, `ROSArduinoBridge.ino`, `commands.h` | **Upload** again |
| Buddy URDF / `controller.yaml` / `ros2_control.xacro` / `ekf.yaml` only | `colcon build --packages-select diffdrive_arduino buddy` on Pi — no ESP reflash for gain-only xacro changes |
| Serial protocol or cross flags in `buddy_robot_config.h` | Re-flash + note in buddy `DRIVE_TRAIN.md` |
| Comment out `USE_IMU` | Re-flash; launch buddy with `use_imu:=false` (or Pi will get `Invalid Command` on `f`) |

## PlatformIO (optional)

Not included by default. Arduino IDE matches buddy documentation.
