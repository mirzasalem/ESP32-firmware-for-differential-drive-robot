# ESP32-firmware-for-differential-drive-robot

ESP32 firmware for **[Buddy](https://github.com/mirzasalem/buddy-ros2)** — a differential-drive mobile robot **designed and built by [Mirza Salem](https://github.com/mirzasalem/)**. This sketch drives L298N motors and quadrature encoders, and optionally reads an MPU9250 gyro, over a serial bridge for ROS 2 (`diffdrive_arduino` / `ros2_control`).

**Verified with Buddy:** velocity-PID closed-loop (`m L R`), integral ramp breakaway at **Ki=150**, `BUDDY_LEFT_ENCODER_INVERT 1`, write-on-change serial + 10 Hz encoder poll — keyboard teleop, in-place turns under body weight, and Nav2 goals reach the target on the current indoor tuning.

**MPU9250 (bench verified):** serial **`g`** / **`f`** / **`i`**, I2C on GPIO 21/22. On the Pi, enable fusion with **`use_imu:=true`** (buddy `robot.launch.py`). Default off on the Pi side. Full setup: **[MPU9250 IMU](#mpu9250-imu-optional)** below.

This repository is **not** a ROS 2 package. Flash the sketch on the ESP32, then run **[buddy-ros2](https://github.com/mirzasalem/buddy-ros2)** on the host (Ubuntu 24.04 + ROS 2 Jazzy).

## Related repositories

| Repo | Role |
|------|------|
| **[buddy-ros2](https://github.com/mirzasalem/buddy-ros2)** | ROS 2: URDF, launches, Nav2, teleop, `ros2_control` |
| **diffdrive_arduino** | Hardware plugin: serial ↔ ESP32 (`ros2_ws/src/diffdrive_arduino`) |
| **[esp2ros2](https://github.com/mirzasalem/esp2ros2)** (this repo) | ESP32 sketch: motors, encoders, IMU, serial protocol |

Typical layout:

```
~/esp/esp2ros2/                    ← flash from here
~/ros2_ws/src/buddy/
~/ros2_ws/src/diffdrive_arduino/
```

Copy the **whole workspace** to a Pi when deploying — do not mix upstream `diffdrive_arduino` with a tweaked local copy.

## Firmware source code

Open **`firmware/ROSArduinoBridge/ROSArduinoBridge.ino`** in Arduino IDE — all tabs load from this folder.

| File | Role |
|------|------|
| **`ROSArduinoBridge.ino`** | Main loop: serial parser, command dispatch, `USE_IMU` define, auto-stop timer |
| **`commands.h`** | Command IDs: `READ_ENCODERS` (`e`), `READ_STATE` (`f`), `READ_IMU` (`g`), `IMU_CALIBRATE` (`i`), `MOTOR_SPEEDS` (`m`), `MOTOR_RAW_PWM` (`o`), `UPDATE_PID` (`u`) |
| **`buddy_robot_config.h`** | Buddy flags: `ENCODER_CROSS 0`, `MOTOR_CROSS 1`, `LEFT_ENCODER_INVERT 1` |
| **`motor_driver.h` / `.ino`** | L298 GPIO 18/19/32/33; `setMotorSpeeds()` applies motor cross |
| **`encoder_driver.h` / `.ino`** | ESP32Encoder on LEFT 26/27, RIGHT 16/17; `readEncoderRosLeft/Right()` |
| **`diff_controller.h`** | Velocity PID (50 Hz): P/D/I/Ko, integral ramp breakaway — no fixed PWM floor |
| **`imu_driver.h` / `.ino`** | MPU9250 raw I2C (SDA 21, SCL 22): 100 Hz sample, gyro bias at boot, serial milli-units |
| **`servos.h` / `.ino`** | Optional servos (`USE_SERVOS` — default off) |

### Key `#define` switches (`ROSArduinoBridge.ino`)

| Define | Buddy default | Effect |
|--------|---------------|--------|
| `L298_MOTOR_DRIVER` | **on** | L298N H-bridge driver |
| `ESP32_ENC_COUNTER` | **on** | Quadrature encoders via ESP32Encoder library |
| `USE_IMU` | **on** | MPU9250 on I2C; enables `f` / `g` / `i`. Comment out if no IMU wired |
| `USE_SERVOS` | off | Servo commands disabled |

Protocol details: [docs/SERIAL_PROTOCOL.md](docs/SERIAL_PROTOCOL.md). Upload guide: [docs/ARDUINO_SETUP.md](docs/ARDUINO_SETUP.md).

## Contents

```
esp2ros2/
├── README.md
├── NOTICE.md
├── firmware/ROSArduinoBridge/   ← open ROSArduinoBridge.ino (all tabs here)
│   └── imu_driver.h / .ino      ← MPU9250 over raw I2C (no extra library)
├── docs/
│   ├── ARDUINO_SETUP.md
│   ├── WIRING.md                ← pins, split motor/encoder cross, IMU I2C
│   └── SERIAL_PROTOCOL.md       ← includes f / g / i (IMU)
├── scripts/
│   ├── test_motors.sh
│   ├── test_motors_diag.sh
│   └── test_closed_loop.sh      ← closed-loop m command bench test
└── libraries/README.md            ← ESP32Encoder
```

## Default hardware profile (buddy)

| Item | Setting |
|------|---------|
| MCU | ESP32, USB serial |
| Motor driver | **L298N** (`L298_MOTOR_DRIVER`) |
| Encoders | **ESP32Encoder** (`ESP32_ENC_COUNTER`) |
| IMU (optional) | **MPU9250** on I2C: SDA **21**, SCL **22**, 3.3 V — `USE_IMU` in `ROSArduinoBridge.ino` |
| Motor GPIO | IN1–4: 18, 19, 32, 33 — see [docs/WIRING.md](docs/WIRING.md) |
| Encoder GPIO | LEFT **26, 27** (`BUDDY_LEFT_ENCODER_INVERT 1`); RIGHT **16, 17** |
| Encoder cross | **`BUDDY_L298_ENCODER_CROSS 0`** — direct map for TF/odom |
| Left encoder invert | **`BUDDY_LEFT_ENCODER_INVERT 1`** — required so PID input matches `m` sign |
| Motor cross | **`BUDDY_L298_MOTOR_CROSS 1`** — L298 PWM crossed on chassis |
| Serial | **115200**, commands end with **CR** |
| Buddy device | `/dev/ttyACM0`, `/dev/ttyACM1`, or stable by-id (lidar often `ttyUSB0`) |
| ROS motor mode | Closed-loop velocity PID: serial **`m L R`** (ticks per frame); PWM ramps from the **I term** |
| Default PID (buddy) | **`u 100:40:150:50`** (P:D:I:Ko) — `pid_i` / Ki is breakaway / ramp under load (raised for turns) |
| Open-loop fallback | Serial **`o L R`**, buddy caps **130–230** (bench only) |
| Pi hardware loop | **20 Hz** — `m` write-on-change + **500 ms** keep-alive (ESP auto-stop **2 s**) |
| IMU serial cost | **None extra** — with the IMU on, the Pi polls `f` (encoders + gyro + accel) instead of `e` |

## Quick start

1. [docs/ARDUINO_SETUP.md](docs/ARDUINO_SETUP.md) — IDE, library, upload  
2. [docs/WIRING.md](docs/WIRING.md) — power, L298, encoders, split cross  
3. Upload **`firmware/ROSArduinoBridge/ROSArduinoBridge.ino`**  
4. On the robot:

   ```bash
   cd ~/ros2_ws && source install/setup.bash
   ros2 launch buddy robot_mapping.launch.py
   ```

Buddy tuning: `ros2_ws/src/buddy/docs/DRIVE_TRAIN.md`.

## MPU9250 IMU (optional)

The firmware reads a **MPU9250** (or MPU6500 module) over I2C and exposes gyro + accel on the serial bus. The Pi fuses **gyro yaw rate** with **wheel forward speed** via `robot_localization` when buddy is launched with **`use_imu:=true`**.

### How firmware integrates with buddy

```text
MPU9250 ──I2C (SDA 21 / SCL 22)──► imu_driver.ino (100 Hz, bias at boot)
                                        │
                    ROSArduinoBridge.ino dispatches:
                    g  → IMU only
                    f  → encoders + IMU (one USB round trip)
                    i  → gyro bias recalibration (~0.5 s, robot still)
                                        │
                                        ▼
                              diffdrive_arduino (Pi)
                              enable_imu → polls "f" instead of "e"
                                        │
                    ┌───────────────────┴───────────────────┐
                    ▼                                       ▼
         diff_drive_controller                    imu_sensor_broadcaster
         /diff_drive_controller/odom            /imu_sensor_broadcaster/imu
                    │                                       │
                    └──────────────► ekf_filter_node ◄──────┘
                                   (buddy config/ekf.yaml)
                                           │
                              /odom (fused) + odom → base_link TF
```

| Firmware | Pi (buddy / diffdrive_arduino) |
|----------|--------------------------------|
| `#define USE_IMU` in `ROSArduinoBridge.ino` | Launch arg `use_imu:=true` |
| Serial poll **`f`** when Pi has IMU on | `enable_imu: true` in `ros2_control.xacro` |
| Raw chip axes (no sign flip in firmware) | Mount rotation in `description/chassis.xacro` + `imu.xacro` |
| Gyro bias at boot + command **`i`** | `imu_calibrate_on_activate: true` on activate |
| Magnetometer **not read** | EKF fuses **`vyaw`** only; wheels provide **`vx`** |

**Critical (Pi side):** EKF must subscribe to **`/diff_drive_controller/odom`**, not `/odom`. See buddy `config/ekf.yaml`.

### One-time setup (firmware + wiring)

1. Wire MPU9250: VCC **3.3 V**, GND, SDA **21**, SCL **22** — [docs/WIRING.md](docs/WIRING.md#mpu9250-imu--esp32-i2c-optional)
2. Keep `#define USE_IMU` enabled in `ROSArduinoBridge.ino`
3. Upload sketch — [docs/ARDUINO_SETUP.md](docs/ARDUINO_SETUP.md)
4. Bench-test with ROS **stopped**:

```bash
~/esp/esp2ros2/scripts/test_closed_loop.sh /dev/ttyACM0 2
```

Pass example — last field of **`g`** must be **`1`**:

```text
g -> 0 -1 -1  151 131 9761  1
```

5. On the Pi: `sudo apt install ros-jazzy-robot-localization ros-jazzy-imu-sensor-broadcaster`
6. Build buddy: `colcon build --packages-select diffdrive_arduino buddy --symlink-install`

Do **not** launch `use_imu:=true` until step 4 passes.

### Run on the robot (with IMU)

| Step | Command / action |
|------|------------------|
| 1 | Confirm bench **`g`** → trailing **`1`** (close Serial Monitor / stop ROS) |
| 2 | `source ~/ros2_ws/install/setup.bash` |
| 3 | Launch navigation with IMU (below) |
| 4 | Wait **~20 s** |
| 5 | `ros2 run buddy keyboard_teleop` → **`k`** then **`i`** — RViz drives forward in facing direction |
| 6 | Optional: Nav2 goal after **2D Pose Estimate** |

**Run this:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch buddy robot_navigation.launch.py \
  map:=/home/mirza/ros2_ws/src/buddy/maps/my_map.yaml \
  use_imu:=true
```

**Verify topics (second terminal):**

```bash
ros2 topic hz /imu_sensor_broadcaster/imu    # ~10 Hz
ros2 topic hz /odom                            # ~20 Hz fused
ros2 topic echo --once /imu_sensor_broadcaster/imu  # angular_velocity.z ≈ 0 while still
```

Full Pi-side guide: [buddy README — MPU9250 IMU](https://github.com/mirzasalem/buddy-ros2/blob/main/README.md#mpu9250-imu-optional) and [buddy/docs/DRIVE_TRAIN.md §9](https://github.com/mirzasalem/buddy-ros2/blob/main/docs/DRIVE_TRAIN.md#9-imu-and-odometry-fusion--mpu9250-optional).

### IMU serial commands (firmware)

| Cmd | Reply | When used |
|-----|-------|-----------|
| **`g`** | `gx gy gz ax ay az imu_ok` | Bench test, debug (gyro mrad/s, accel mm/s²) |
| **`f`** | `left right gx gy gz ax ay az imu_ok` | Pi polls this every 2nd frame when `enable_imu` (same cost as `e`) |
| **`i`** | `OK` or `IMU FAIL` | Re-estimate gyro bias — robot must be still |

If `USE_IMU` is commented out, **`g`** / **`f`** return `Invalid Command` or `imu_ok 0`.

## Buddy ↔ firmware sync

| Buddy file | Must match firmware |
|------------|---------------------|
| `description/ros2_control.xacro` | `device`, `baud_rate`, `enc_counts_per_rev`, `use_open_loop_pwm` (false), `loop_rate` (**20**), `encoder_read_divisor` (**2**), `motor_keepalive_ms` (**500**), PID **100/40/150/50**, `enable_imu` / `imu_sensor_name` when `use_imu:=true` |
| `description/imu.xacro` + `chassis.xacro` | `imu_link` mount rpy/xyz — chip frame in firmware; rotation only in URDF |
| `config/ekf.yaml` | EKF reads **`/diff_drive_controller/odom`** + `/imu_sensor_broadcaster/imu`; publishes fused **`/odom`** |
| `launch/robot.launch.py` | `use_imu:=true` → EKF + `imu_sensor_broadcaster`; else **`odom_relay`** → `/odom` |
| `description/drive_train.xacro` | `encoder_counts_per_rev`, wheel size |
| `config/controller.yaml` | `wheel_separation`, `wheel_radius`, accel caps — **only** host-side ramp |
| `config/nav2_params.yaml` | RPP + velocity_smoother caps aligned with `controller.yaml` |

Nav2 also needs `twist_stamper` on the Pi (buddy launch) — not part of this repo.

After encoder or gear changes: update `encoder_counts_per_rev`, re-flash if GPIO or cross flags changed, **re-map** if odom changed.

## Test without ROS

**Closed-loop (buddy default — flash firmware first):**

```bash
~/esp/esp2ros2/scripts/test_closed_loop.sh /dev/ttyACM0 3
```

Pass: wheels **ease in** (no PWM snap at 130), encoders move on `m 5 5`, hold wheel → pushes harder, clean stop on `m 0 0`. IMU section: **`g`** last field **`1`**, gyro ≈ 0 while still, **`az` ≈ 9800**.

**With ROS running:** buddy launch log shows `closed-loop m` with non-zero ticks on turns; Nav2 goals complete when AMCL pose matches the map.

**Open-loop fallback:**

```bash
~/esp/esp2ros2/scripts/test_motors.sh /dev/ttyUSB1 130 130 2
~/esp/esp2ros2/scripts/test_motors_diag.sh /dev/ttyUSB1
```

Close Serial Monitor and buddy before using the port.

## Troubleshooting

| Problem | What to do |
|---------|------------|
| Upload fails | Data USB cable; correct port; ESP32 drivers |
| `ESP32Encoder.h` missing | [libraries/README.md](libraries/README.md) |
| One Arduino tab only | Open `firmware/ROSArduinoBridge/ROSArduinoBridge.ino` |
| Forward OK, **turns reversed** (Nav2 or `j`/`l`) | Re-flash with `ENCODER_CROSS=0`, `MOTOR_CROSS=1`; keep `swap_motor_pwm: false` on Pi — see [WIRING.md](docs/WIRING.md) |
| One wheel backward on `i` only | Swap that motor’s two wires, or `motor_scale` −1.0 on that side in `ros2_control.xacro` |
| RViz wheel TF wrong vs robot | Check `ENCODER_CROSS` and encoder GPIO — see [WIRING.md](docs/WIRING.md) |
| Buddy no `/odom` | `diff_drive_controller` active; one process on serial port |
| PWM below 130 no spin (open-loop) | Normal — buddy `open_loop_min_pwm` is **130**; use `test_motors.sh 130 …` |
| Left wheel spins away on turns / `m 5 5` left ticks opposite sign | Re-flash with `BUDDY_LEFT_ENCODER_INVERT 1`; Pi `negate_left_encoder_odom: false` |
| Wheels snap / over-turn | Old firmware with PWM-130 floor — re-flash; confirm `diff_controller.h` uses integral PID |
| Auto-stop after 2 s | Normal without commands; buddy resends `m` every **500 ms** keep-alive when cruising |
| `test_closed_loop.sh` syntax error on delta | Update script (strips `\r` from encoder lines) |
| `g` returns `Invalid Command` | Firmware predates the IMU, or `USE_IMU` is commented out in `ROSArduinoBridge.ino` — re-flash |
| `g` last field is `0` | ESP found nothing on I2C: check SDA **21**, SCL **22**, 3.3 V, common ground ([WIRING.md](docs/WIRING.md)) |
| Gyro non-zero while parked | Bias sampled while moving at power-on — send `i`, or relaunch (Pi recalibrates on activate) |
| RViz turns with gyro but forward misaligned on map | Pi EKF must read **`/diff_drive_controller/odom`** for **`vx`** — see buddy `config/ekf.yaml` |

## Deploying to Raspberry Pi

1. Copy `~/esp/esp2ros2` and `~/ros2_ws/src` (buddy + diffdrive_arduino + deps).  
2. Rebuild on the Pi: `colcon build --packages-select diffdrive_arduino buddy …`
3. Re-flash ESP from laptop or Pi if needed.  
4. Run `buddy/scripts/setup_usb_serial.sh` once.

## License

See [NOTICE.md](NOTICE.md). Based on [ROSArduinoBridge](https://github.com/hbrobotics/ros_arduino_bridge) (BSD).

## 👨 Author

Mirza Salem  
[GitHub](https://github.com/mirzasalem/) | [LinkedIn](https://www.linkedin.com/in/mirzasalem/) | [Portfolio](https://mirzasalem.vercel.app/)
