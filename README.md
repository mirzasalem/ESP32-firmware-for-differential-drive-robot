# ESP32-firmware-for-differential-drive-robot

ESP32 firmware for **[Buddy](https://github.com/mirzasalem/buddy-ros2)** — a differential-drive mobile robot **designed and built by [Mirza Salem](https://github.com/mirzasalem/)**. This sketch drives L298N motors and quadrature encoders, and optionally reads an MPU9250 gyro, over a serial bridge for ROS 2 (`diffdrive_arduino` / `ros2_control`).

**Verified with Buddy:** velocity-PID closed-loop (`m L R`), integral ramp breakaway (raised 2026-09-02 to `Ki=350`, floor-verified with the robot's real weight on the wheels — see [DRIVE_TRAIN.md](https://github.com/mirzasalem/buddy-ros2/blob/main/docs/DRIVE_TRAIN.md)), `BUDDY_LEFT_ENCODER_INVERT 1`, write-on-change serial + 10 Hz encoder poll — keyboard teleop, in-place turns under body weight, and Nav2 goals reach the target on the current indoor tuning.

**MPU9250 path (`USE_IMU`, commands `f` / `g` / `i`):** bench-verified on hardware (serial `g` → `imu_ok 1`, gyro ≈ 0 while still, `az` ≈ 9800 mm/s²). Off by default on the Pi side (`use_imu:=false`) — the drive stack above works identically either way; flip `use_imu:=true` at launch to enable gyro-fused heading. See DRIVE_TRAIN.md §9 for the full setup.

This repository is **not** a ROS 2 package. Flash the sketch on the ESP32, then run **[buddy-ros2](https://github.com/mirzasalem/buddy-ros2)** on the host (Ubuntu 24.04 + ROS 2 Jazzy).

## Related repositories

| Repo | Role |
|------|------|
| **[buddy-ros2](https://github.com/mirzasalem/buddy-ros2)** | ROS 2: URDF, launches, Nav2, teleop, `ros2_control` |
| **diffdrive_arduino** | Hardware plugin: serial ↔ ESP32 (`ros2_ws/src/diffdrive_arduino`) |
| **[esp2ros2](https://github.com/mirzasalem/esp2ros2)** (this repo) | ESP32 sketch: motors, encoders, protocol |

Typical layout:

```
~/esp/esp2ros2/                    ← flash from here
~/ros2_ws/src/buddy/
~/ros2_ws/src/diffdrive_arduino/
```

Copy the **whole workspace** to a Pi when deploying — do not mix upstream `diffdrive_arduino` with a tweaked local copy.

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
│   ├── test_closed_loop.sh      ← closed-loop m command bench test
│   ├── pwm_sweep.py             ← open-loop breakaway + low-target tracking sweep
│   └── calibrate_360.py         ← gyro-referenced encoder / track calibration
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
| Default PID (buddy) | **`u 100:80:350:50`** (P:D:I:Ko) — `pid_i` / Ki is breakaway / ramp under load (raised 2026-09-02, floor-verified) |
| Open-loop fallback | Serial **`o L R`**, buddy caps **130–230** (bench only) |
| Pi hardware loop | **20 Hz** — `m` write-on-change + **500 ms** keep-alive (ESP auto-stop **2 s**) |
| ESP `PID_RATE` | **20 Hz** — must match Pi `loop_rate`. A 50 Hz MCU with a 20 Hz Pi overspeeds (~2.3× linear `odom/cmd`) |
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

## Buddy ↔ firmware sync

| Buddy file | Must match firmware |
|------------|---------------------|
| `description/ros2_control.xacro` | `device`, `baud_rate`, `enc_counts_per_rev`, `use_open_loop_pwm` (false), `loop_rate` (**20**), `encoder_read_divisor` (**2**), `motor_keepalive_ms` (**500**), PID **100/80/350/50**, `motor_scale` (1.0) |
| `description/imu.xacro` + `chassis.xacro` | `imu_link` mount rpy/xyz — must describe how the MPU9250 is actually bolted on; firmware reports raw chip axes |
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

Pass: wheels ease in (no PWM snap at 130), encoders move on `m 5 5`, hold wheel → pushes harder, clean stop on `m 0 0`.

**With ROS running:** buddy launch log shows `closed-loop m` with non-zero ticks on turns; Nav2 goals complete when AMCL pose matches the map.

**Open-loop fallback:**

```bash
~/esp/esp2ros2/scripts/test_motors.sh /dev/ttyUSB1 130 130 2
~/esp/esp2ros2/scripts/test_motors_diag.sh /dev/ttyUSB1

# Encoder / track-width calibration, gyro-referenced (needs USE_IMU).
# Spins in place, prints ticks per wheel rev, per robot 360, and per metre.
python3 ~/esp/esp2ros2/scripts/calibrate_360.py /dev/ttyACM0

# Open-loop PWM breakaway + closed-loop low-target tracking sweep.
python3 ~/esp/esp2ros2/scripts/pwm_sweep.py /dev/ttyACM0
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
| PWM below 130 no spin (open-loop) | Not necessarily a fault — buddy `open_loop_min_pwm` is **130** so ROS never sends less, but a 2026-09-02 loaded bench measured actual breakaway around PWM 75-80, so lower values on `test_motors.sh` may still spin. If it truly doesn't move even at 130+, check wiring/driver, not the PWM value. |
| Left wheel spins away on turns / `m 5 5` left ticks opposite sign | Re-flash with `BUDDY_LEFT_ENCODER_INVERT 1`; Pi `negate_left_encoder_odom: false` |
| Wheels snap / over-turn | Old firmware with PWM-130 floor — re-flash; confirm `diff_controller.h` uses integral PID |
| Auto-stop after 2 s | Normal without commands; buddy resends `m` every **500 ms** keep-alive when cruising |
| `test_closed_loop.sh` syntax error on delta | Update script (strips `\r` from encoder lines) |
| `g` returns `Invalid Command` | Firmware predates the IMU, or `USE_IMU` is commented out in `ROSArduinoBridge.ino` — re-flash |
| `g` last field is `0` | ESP found nothing on I2C: check SDA **21**, SCL **22**, 3.3 V, common ground ([WIRING.md](docs/WIRING.md)) |
| Gyro reads non-zero while still | Robot was moving at power-on (bias is sampled then) — send `i`, or let the Pi do it on activate |

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
