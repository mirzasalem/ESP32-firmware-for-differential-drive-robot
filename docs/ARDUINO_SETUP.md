# Firmware Upload Guide (Arduino IDE)

This guide covers installing toolchain dependencies and uploading `ROSArduinoBridge` to an ESP32.

Buddy ships with firmware tuned and **verified on the real robot**. Upload this tree before the first drive, run the closed-loop bench test, then launch buddy navigation.

Related documents:

| Document | Contents |
|----------|----------|
| [../README.md](../README.md) | Project overview, wiring summary, architecture |
| [WIRING.md](WIRING.md) | Power, L298N, encoders, MPU9250 |
| [SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md) | Serial command reference |

---

## 1. Install Arduino IDE

Install [Arduino IDE 2.x](https://www.arduino.cc/en/software) for Linux, Windows, or macOS.

---

## 2. Add ESP32 board support

1. **File → Preferences → Additional boards manager URLs** — add:

   ```text
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```

2. **Tools → Board → Boards Manager** → search **esp32** → install **esp32** by Espressif.
3. **Tools → Board → esp32 → ESP32 Dev Module** (or the matching module).

Recommended **Tools** settings:

| Setting | Value |
|---------|--------|
| Upload speed | 921600 (use 115200 if upload is unreliable) |
| USB CDC On Boot | Enabled (if the board uses native USB CDC) |
| Flash size | Match the module (commonly 4 MB) |

---

## 3. Install the ESP32Encoder library

Required for quadrature encoder counting. See [../libraries/README.md](../libraries/README.md).

**Library Manager:** Sketch → Include Library → Manage Libraries → search **ESP32Encoder** → install (0.11.7 or newer tested).

**Or copy:**

```bash
cp -r /path/to/ESP32Encoder ~/Arduino/libraries/ESP32Encoder
```

Restart the IDE after installing.

---

## 4. Open the sketch

**File → Open** → select:

```text
firmware/ROSArduinoBridge/ROSArduinoBridge.ino
```

All sketch tabs must appear (`motor_driver.ino`, `encoder_driver.ino`, `imu_driver.ino`, `diff_controller.h`, …).  
If only one tab is visible, the wrong path was opened.

### Firmware modules

| File | Purpose |
|------|---------|
| `ROSArduinoBridge.ino` | Serial command loop, feature defines |
| `commands.h` | Command letter → handler IDs |
| `buddy_robot_config.h` | Encoder/motor mapping flags |
| `motor_driver.*` | L298N PWM on GPIO 18/19/32/33 |
| `encoder_driver.*` | ESP32Encoder; ROS left/right reads |
| `diff_controller.h` | Velocity PID (ROS1 joey_v1 port) at `PID_RATE` **20 Hz**, which must match buddy's ros2_control `loop_rate`. PWM starts from 0 and **Ki** ramps it under load (no PWM-130 floor) |
| `imu_driver.*` | MPU9250 raw I2C (optional): ±250 °/s, 100 Hz, gyro bias averaged at boot. Magnetometer unused; the mount orientation belongs in buddy `chassis.xacro` (`chassis_imu_*`) |

Re-upload after editing any of these files.

---

## 5. Verify build configuration

Near the top of `ROSArduinoBridge.ino`:

```cpp
#define L298_MOTOR_DRIVER
#define ESP32_ENC_COUNTER
//#define CYTRON_MDD3A
#define USE_IMU
```

| Define | Meaning |
|--------|---------|
| `L298_MOTOR_DRIVER` | Use L298N (default) |
| `ESP32_ENC_COUNTER` | Hardware encoder path |
| `USE_IMU` | Enable MPU9250 on I2C (SDA 21 / SCL 22) |

Enable only one motor-driver define. Comment out `USE_IMU` if no IMU is fitted; `f` then reports `imu_ok 0`.

`USE_IMU` needs no extra library. `imu_driver.ino` talks to the registers directly through `Wire`, so it also works with the MPU6500 boards sold as MPU9250.

### Mapping defaults (`buddy_robot_config.h`)

| Option | Default |
|--------|---------|
| `BUDDY_L298_ENCODER_CROSS` | `0` |
| `BUDDY_L298_MOTOR_CROSS` | `1` |
| `BUDDY_LEFT_ENCODER_INVERT` | `1` |

Details: [WIRING.md](WIRING.md).

### IMU wiring reminder

| MPU9250 | ESP32 |
|---------|-------|
| VCC | 3.3 V |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

Keep the robot stationary at USB power-on so gyro bias can be averaged (200 samples).

---

## 6. Select port and upload

1. Connect the ESP32 with a **data-capable** USB cable.
2. **Tools → Port** → `/dev/ttyACM0`, `/dev/ttyACM1`, or `/dev/ttyUSB*`.
3. Click **Upload**.
4. Wait for **Done uploading**.

### Linux permissions

```bash
sudo usermod -aG dialout $USER
# log out and back in
```

---

## 7. Post-upload verification

**Tools → Serial Monitor** → baud **115200** → line ending **Carriage return** (or Both NL & CR).

| Command | Expected |
|---------|----------|
| `b` | `115200` |
| `u 100:80:350:50` | `OK` (buddy default PID, P:D:I:Ko; raised 2026-09-02, floor-verified) |
| `r` | `OK` (reset encoders + PID) |
| `e` | Two integers (left, right counts) |
| `m 5 5` | Motion — **no reply** |
| `m 0 0` | Stop |
| `o 100 100` | `OK 100 100` |
| `o 100 -100` | Spin in place (if the wiring matches buddy) |
| `o 0 0` | Stop |
| `g` | `gx gy gz ax ay az 1` when IMU is present |
| `i` | `OK` (gyro bias recalibration; robot still) |

### Automated bench test

Closed loop (recommended after every upload):

```bash
./scripts/test_closed_loop.sh /dev/ttyACM0 3
```

Open-loop wiring check:

```bash
./scripts/test_motors_diag.sh /dev/ttyACM0
```

Close the Serial Monitor before running scripts or ROS 2 (exclusive port access).

### IMU acceptance check

With ROS stopped and the robot still, send `g`:

```text
g -> 0 -1 -1  151 131 9761  1
                    │         └── imu_ok must be 1
                    └── az ≈ 9800 mm/s² (board flat)
```

| Result | Action |
|--------|--------|
| Trailing `1` | IMU OK |
| Trailing `0` | Check I2C wiring ([WIRING.md](WIRING.md)) |
| `Invalid Command` | Re-flash with `#define USE_IMU` |
| Large gyro while still | Send `i`, or power-cycle while stationary |

Combined state read (host path when IMU is enabled):

```bash
python3 -c "import serial,time; s=serial.Serial('/dev/ttyACM0',115200,timeout=1); time.sleep(2); s.write(b'f\r'); time.sleep(0.2); print(s.read(128).decode())"
```

Reply: `left right gx gy gz ax ay az imu_ok`

---

## 8. Host bring-up (ROS 2)

After firmware verification:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch buddy robot_mapping.launch.py
```

With IMU fusion (after IMU bench pass):

```bash
sudo apt install ros-jazzy-robot-localization ros-jazzy-imu-sensor-broadcaster
colcon build --packages-select diffdrive_arduino buddy --symlink-install
source install/setup.bash
ros2 launch buddy robot_navigation.launch.py use_imu:=true
```

| Mode | ESP poll | `/odom` source |
|------|----------|----------------|
| `use_imu:=false` | `e` | Relay from `diff_drive_controller` |
| `use_imu:=true` | `f` | EKF (wheel `vx` + gyro `vyaw`) |

**Navigation check (optional):** `ros2 launch buddy robot_navigation.launch.py map:=...`, wait ~20 s, then send a Nav2 goal. The hardware log should show non-zero `closed-loop m` values during turns, and the goal should succeed once the AMCL pose is correct.

---

## 9. When to re-flash

| Change | Action |
|--------|--------|
| Firmware sources (`motor_driver`, `encoder_driver`, `imu_driver`, PID, protocol) | Re-upload |
| Host-only YAML / URDF / PID xacro gains | Rebuild host packages — no ESP reflash |
| Mapping flags in `buddy_robot_config.h` | Re-upload |
| Disable `USE_IMU` | Re-upload; launch host with `use_imu:=false` |

---

## PlatformIO

Not provided by default. Arduino IDE is the documented path for this repository.
