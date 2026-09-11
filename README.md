# ESP32 Firmware for Differential-Drive Robots

ESP32 firmware that bridges **motors, quadrature encoders, and an optional MPU9250 IMU** to a host computer over **USB serial**. Designed for ROS 2 (`ros2_control` / `diffdrive_arduino`) on Ubuntu 24.04 with ROS 2 Jazzy.

This repository contains **embedded firmware only** — it is not a ROS 2 package. Flash the sketch onto an ESP32, then run the host stack from **[buddy-ros2](https://github.com/mirzasalem/buddy-ros2)**.

| | |
|---|---|
| **MCU** | ESP32 (USB CDC serial, 115200 baud) |
| **Actuation** | L298N H-bridge, closed-loop velocity PID at 20 Hz (matches the host `loop_rate`) |
| **Odometry** | Quadrature encoders via ESP32Encoder (PCNT) |
| **Inertial sensing** | MPU9250 / MPU6500 over I2C (100 Hz), optional |
| **Host interface** | Text protocol over USB (`e`, `f`, `g`, `m`, `o`, `u`, …) |

**Author:** [Mirza Salem](https://github.com/mirzasalem/) · [LinkedIn](https://www.linkedin.com/in/mirzasalem/) · [Portfolio](https://mirzasalem.vercel.app/)

**Verified with Buddy:** closed-loop velocity PID (`m L R`), integral-ramp breakaway (raised 2026-09-02 to `Ki=350`, floor-verified with the robot's real weight on the wheels, see [DRIVE_TRAIN.md](https://github.com/mirzasalem/buddy-ros2/blob/main/docs/DRIVE_TRAIN.md)), `BUDDY_LEFT_ENCODER_INVERT 1`, write-on-change serial + 10 Hz encoder poll. Keyboard teleop, in-place turns under body weight, and Nav2 goals all reach the target on the current indoor tuning.

**MPU9250 path (`USE_IMU`, commands `f` / `g` / `i`):** bench-verified on hardware (`g` → `imu_ok 1`, gyro ≈ 0 while still, `az` ≈ 9800 mm/s²). Buddy's launch files default to `use_imu:=false` and the drive stack works the same either way; `use_imu:=true` adds gyro-fused heading. Full setup: buddy DRIVE_TRAIN.md §9.

---

## Architecture

```text
  Wheel encoders ──GPIO──┐
                         ├──► ESP32 ──USB serial (115200)──► Host (ROS 2)
  MPU9250 IMU ────I2C────┘      │                              │
                                │  20 Hz velocity PID          │  ros2_control
                                │  100 Hz IMU sample           │  EKF: vx + vyaw → /odom
                                ▼
                           L298N → motors
```

Encoders and the IMU are sampled on the microcontroller. The host never accesses I2C or GPIO directly. When IMU fusion is enabled, a single serial poll (`f`) returns encoder counts and inertial data in one USB round-trip.

---

## Related repositories

| Repository | Role |
|------------|------|
| **[buddy-ros2](https://github.com/mirzasalem/buddy-ros2)** | ROS 2: URDF, launches, Nav2, teleop, `ros2_control` |
| **diffdrive_arduino** | Hardware interface plugin: serial ↔ ESP32 |
| **This repository** | ESP32 sketch: motors, encoders, IMU, serial protocol |

---

## Documentation

| Document | Contents |
|----------|----------|
| **[docs/WIRING.md](docs/WIRING.md)** | Power, L298N, encoders, MPU9250 pin map |
| **[docs/ARDUINO_SETUP.md](docs/ARDUINO_SETUP.md)** | Arduino IDE setup and firmware upload |
| **[docs/SERIAL_PROTOCOL.md](docs/SERIAL_PROTOCOL.md)** | USB serial command reference |
| **[libraries/README.md](libraries/README.md)** | Required Arduino libraries |

---

## Hardware and wiring

### Default pin assignment

| Function | Connection | ESP32 pins |
|----------|------------|------------|
| Motor driver (L298N) | IN1–IN4 | **18, 19, 32, 33** |
| Left encoder | Channels A/B | **26, 27** |
| Right encoder | Channels A/B | **16, 17** |
| IMU (MPU9250) | I2C SDA / SCL | **21, 22** |
| Host link | USB | CDC serial @ **115200** |

### Power

| Rail | Connection |
|------|------------|
| Motor supply | Battery **7–12 V** → L298N `+12V` / `VMS` |
| Logic / serial | ESP32 powered from **USB** (PC or Raspberry Pi) |
| Ground | **Common GND** required: ESP32 ↔ L298N ↔ battery |

Do **not** power motors from the ESP32 3.3 V or 5 V pins.

### Motor driver (L298N)

| L298N | ESP32 GPIO |
|-------|------------|
| IN1 | 18 |
| IN2 | 19 |
| IN3 | 32 |
| IN4 | 33 |
| ENA / ENB | Jumper to 5 V, or GPIO 25 / 14 |
| GND | Common ground |
| OUT1–OUT2 | Left wheel motor |
| OUT3–OUT4 | Right wheel motor |

### Encoders

| Side | GPIO A | GPIO B | Notes |
|------|--------|--------|-------|
| Left | 26 | 27 | 3.3 V logic; internal pull-ups enabled; `BUDDY_LEFT_ENCODER_INVERT 1` (required so the PID input matches the `m` sign) |
| Right | 16 | 17 | Same, counts used as-is |

### IMU (MPU9250, optional)

| MPU9250 | ESP32 |
|---------|-------|
| VCC | **3.3 V** |
| GND | GND |
| SDA | **21** |
| SCL | **22** |

Magnetometer data are not read. Keep the robot stationary at power-on so gyro bias can be estimated.

### System overview

```text
                    ┌─────────────┐
   Motor battery ──►│   L298N     ├──► OUT1/2  → left wheel
                    │  IN1..IN4   ├──► OUT3/4  → right wheel
                    └──────┬──────┘
                           │ GPIO 18, 19, 32, 33
                    ┌──────▼──────┐
   Enc L: 26, 27 ──►│   ESP32     │◄── USB ──► Host (ROS 2)
   Enc R: 16, 17 ──►│             │
   MPU9250 ────────►│ SDA 21      │   I2C, 3.3 V (optional)
                    │ SCL 22      │
                    └─────────────┘
```

Full pin notes, chassis cross-mapping, and diagnostics: **[docs/WIRING.md](docs/WIRING.md)**.

---

## Upload firmware to the ESP32

### Prerequisites

1. [Arduino IDE 2.x](https://www.arduino.cc/en/software)
2. ESP32 board support (Boards Manager → **esp32** by Espressif)
3. **ESP32Encoder** library (Library Manager, or see [libraries/README.md](libraries/README.md))
4. Linux serial access (once):

```bash
sudo usermod -aG dialout $USER
# log out and back in
```

### Upload steps

1. Connect the ESP32 with a **data-capable** USB cable.
2. Open the sketch:

   ```
   firmware/ROSArduinoBridge/ROSArduinoBridge.ino
   ```

   All related tabs (`motor_driver`, `encoder_driver`, `imu_driver`, …) must appear. If only one tab opens, the wrong file was selected.
3. Select board: **Tools → Board → ESP32 Dev Module** (or your module).
4. Select port: **Tools → Port** → `/dev/ttyACM0`, `/dev/ttyACM1`, or `/dev/ttyUSB*`.
5. Confirm defines near the top of `ROSArduinoBridge.ino`:

   ```cpp
   #define L298_MOTOR_DRIVER
   #define ESP32_ENC_COUNTER
   #define USE_IMU          // comment out if no IMU is fitted
   ```

6. Click **Upload** and wait for *Done uploading*.

Step-by-step IDE settings and post-upload checks: **[docs/ARDUINO_SETUP.md](docs/ARDUINO_SETUP.md)**.

### Quick serial check

**Tools → Serial Monitor** → **115200** baud → line ending **Carriage return**.

| Command | Expected response |
|---------|-------------------|
| `b` | `115200` |
| `e` | `left right` encoder counts |
| `g` | `gx gy gz ax ay az 1` (IMU present; trailing `1`) |
| `m 5 5` | Wheels move — **no reply** |
| `m 0 0` | Stop |

Automated bench test:

```bash
./scripts/test_closed_loop.sh /dev/ttyACM0 3
```

Close the Serial Monitor before running ROS 2 or the bench scripts (only one process may own the port).

---

## Firmware source

Open `firmware/ROSArduinoBridge/ROSArduinoBridge.ino` in Arduino IDE.

| File | Role |
|------|------|
| `ROSArduinoBridge.ino` | Serial parser, command dispatch, auto-stop |
| `commands.h` | Command IDs (`e`, `f`, `g`, `i`, `m`, `o`, `u`, …) |
| `buddy_robot_config.h` | Encoder/motor mapping flags |
| `motor_driver.*` | L298N GPIO control |
| `encoder_driver.*` | Quadrature counting (ESP32Encoder) |
| `diff_controller.h` | Velocity PID (`PID_RATE` 20 Hz, set in `ROSArduinoBridge.ino`) |
| `imu_driver.*` | MPU9250 raw I2C driver (no extra library) |

### Build switches

| Define | Default | Effect |
|--------|---------|--------|
| `L298_MOTOR_DRIVER` | on | L298N driver |
| `ESP32_ENC_COUNTER` | on | Hardware encoder counting |
| `USE_IMU` | on | Enable MPU9250 (`f` / `g` / `i`) |
| `USE_SERVOS` | off | Servo commands disabled |

---

## Serial protocol (summary)

| Baud | Line ending | Role |
|------|-------------|------|
| 115200 | CR (`\r`) | Host ↔ ESP32 |

| Cmd | Purpose |
|-----|---------|
| `e` | Read encoders |
| `f` | Read encoders **and** IMU (one round-trip) |
| `g` | Read IMU only |
| `i` | Recalibrate gyro bias (robot must be still) |
| `m L R` | Closed-loop speed (ticks per 20 Hz PID frame); no reply |
| `o L R` | Open-loop PWM (buddy caps 130–230, bench only) |
| `u P:D:I:Ko` | Update PID gains (buddy default `u 100:80:350:50`) |

Full specification: **[docs/SERIAL_PROTOCOL.md](docs/SERIAL_PROTOCOL.md)**.

---

## Host settings that must match (buddy)

| Setting | Value |
|---------|-------|
| ESP `PID_RATE` | **20 Hz**, must match the host `loop_rate`. A 50 Hz ESP with a 20 Hz host overspeeds (~2.3× linear `odom/cmd`) |
| Host hardware loop | **20 Hz**, `m` write-on-change + **500 ms** keep-alive (ESP auto-stop **2 s**) |
| Default PID | **`u 100:80:350:50`** (P:D:I:Ko). `pid_i` / Ki is the breakaway ramp under load (raised 2026-09-02, floor-verified) |
| IMU serial cost | **None extra**: with the IMU on, the host polls `f` (encoders + gyro + accel) instead of `e` |
| ESP32 port | `/dev/ttyACM0`, `/dev/ttyACM1`, or stable by-id (lidar is usually `ttyUSB0`) |

| Buddy file | Must match firmware |
|------------|---------------------|
| `description/ros2_control.xacro` | `device`, `baud_rate`, `enc_counts_per_rev`, `use_open_loop_pwm` (false), `loop_rate` (**20**), `encoder_read_divisor` (**2**), `motor_keepalive_ms` (**500**), PID **100/80/350/50**, `motor_scale` (1.0) |
| `description/imu.xacro` + `chassis.xacro` | `imu_link` mount rpy/xyz: must describe how the MPU9250 is actually mounted, since the firmware reports raw chip axes |
| `description/drive_train.xacro` | `encoder_counts_per_rev`, wheel size |
| `config/controller.yaml` | `wheel_separation`, `wheel_radius`, accel caps (the **only** host-side ramp) |
| `config/nav2_params.yaml` | RPP + velocity_smoother caps aligned with `controller.yaml` |

Nav2 also needs `twist_stamper` on the host (buddy launch), which isn't part of this repo.

---

## IMU and odometry fusion

```text
MPU9250 ──I2C──► imu_driver (100 Hz, bias at boot)
                      │
         g → IMU only │  f → encoders + IMU  │  i → bias recalibration
                      ▼
              USB serial to host
                      │
         ┌────────────┴────────────┐
         ▼                         ▼
  diff_drive_controller    imu_sensor_broadcaster
         │                         │
         └──────────► EKF ◄────────┘
                      │
                 /odom (fused)
```

- Firmware reports chip-frame axes in milli-units (gyro: mrad/s, accel: mm/s²).
- Mount orientation is defined in the host URDF, not by sign flips in firmware.
- On the host, launch with `use_imu:=true` after a successful bench `g` reply ending in `1`.

---

## Repository layout

```text
esp2ros2/
├── README.md
├── NOTICE.md
├── firmware/ROSArduinoBridge/     # Arduino sketch (open .ino here)
│   └── imu_driver.h / .ino        # MPU9250 over raw I2C (no extra library)
├── docs/
│   ├── ARDUINO_SETUP.md           # Upload guide
│   ├── WIRING.md                  # Wiring reference (pins, motor/encoder cross, IMU I2C)
│   ├── SERIAL_PROTOCOL.md         # Protocol reference (includes f / g / i)
│   └── buddy_usb_imu_encoder.gif  # Architecture animation
├── scripts/
│   ├── test_closed_loop.sh        # closed-loop m bench test
│   ├── test_motors.sh             # open-loop o
│   ├── test_motors_diag.sh        # open-loop wiring diagnosis
│   ├── pwm_sweep.py               # open-loop breakaway + low-target tracking sweep
│   └── calibrate_360.py           # gyro-referenced encoder / track calibration
└── libraries/README.md
```

---

## Testing without ROS

```bash
# Closed-loop velocity PID (recommended)
./scripts/test_closed_loop.sh /dev/ttyACM0 3

# Open-loop motor wiring check
./scripts/test_motors_diag.sh /dev/ttyACM0
./scripts/test_motors.sh /dev/ttyACM0 130 130 2

# Encoder / track-width calibration, gyro-referenced (needs USE_IMU).
# Spins in place, prints ticks per wheel rev, per robot 360°, and per metre.
python3 ./scripts/calibrate_360.py /dev/ttyACM0

# Open-loop PWM breakaway + closed-loop low-target tracking sweep.
python3 ./scripts/pwm_sweep.py /dev/ttyACM0
```

Pass: the wheels ease in (no PWM snap at 130), the encoders move on `m 5 5`, a wheel held by hand gets pushed harder, and `m 0 0` stops cleanly.

**With ROS running:** the buddy launch log shows `closed-loop m` with non-zero ticks on turns, and Nav2 goals complete when the AMCL pose matches the map.

---

## Troubleshooting

| Issue | Action |
|-------|--------|
| Upload fails | Use a data USB cable; confirm port and ESP32 drivers |
| `ESP32Encoder.h` missing | Install library — [libraries/README.md](libraries/README.md) |
| Only one Arduino tab | Open `firmware/ROSArduinoBridge/ROSArduinoBridge.ino` |
| `g` → trailing `0` | Check IMU wiring (SDA 21, SCL 22, 3.3 V, common GND) |
| `g` → `Invalid Command` | Re-flash with `#define USE_IMU` enabled |
| Motors stop after ~2 s | Expected without keep-alive; host should resend `m` every ≤500 ms |
| Turns reversed | See encoder/motor cross flags in [WIRING.md](docs/WIRING.md) |
| Gyro reads non-zero while still | The robot moved at power-on (bias is sampled then). Send `i`, or let the host do it on activate |
| Left wheel spins away on turns / `m 5 5` left ticks have the opposite sign | Re-flash with `BUDDY_LEFT_ENCODER_INVERT 1`; host `negate_left_encoder_odom: false` |
| Wheels snap / over-turn | Old firmware with the PWM-130 floor. Re-flash, and confirm `diff_controller.h` uses the integral PID |
| Robot ~2.3× too fast in a straight line | ESP `PID_RATE` doesn't match the host `loop_rate` (both must be 20) |
| PWM below 130 doesn't spin (open-loop) | Not necessarily a fault. Buddy's `open_loop_min_pwm` is **130**, so ROS never sends less, but a 2026-09-02 loaded bench measured real breakaway around PWM 75–80. If it won't move even at 130+, check the wiring/driver, not the PWM value |
| `test_closed_loop.sh` syntax error on delta | Update the script (it strips `\r` from encoder lines) |

---

## License

See [NOTICE.md](NOTICE.md). Derived from [ROSArduinoBridge](https://github.com/hbrobotics/ros_arduino_bridge) (BSD).

---

## Author

**Mirza Salem**  
[GitHub](https://github.com/mirzasalem/) · [LinkedIn](https://www.linkedin.com/in/mirzasalem/) · [Portfolio](https://mirzasalem.vercel.app/)
