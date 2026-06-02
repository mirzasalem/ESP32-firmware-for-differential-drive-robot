# ESP32-firmware-for-differential-drive-robot

ESP32 firmware for **[Buddy](https://github.com/mirzasalem/buddy-ros2)** — a differential-drive mobile robot **designed and built by [Mirza Salem](https://github.com/mirzasalem/)**. This sketch drives L298N motors and quadrature encoders over a serial bridge for ROS 2 (`diffdrive_arduino` / `ros2_control`).

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
├── docs/
│   ├── ARDUINO_SETUP.md
│   ├── WIRING.md                ← pins, split motor/encoder cross
│   └── SERIAL_PROTOCOL.md
├── scripts/
│   ├── test_motors.sh
│   └── test_motors_diag.sh
└── libraries/README.md            ← ESP32Encoder
```

## Default hardware profile (buddy)

| Item | Setting |
|------|---------|
| MCU | ESP32, USB serial |
| Motor driver | **L298N** (`L298_MOTOR_DRIVER`) |
| Encoders | **ESP32Encoder** (`ESP32_ENC_COUNTER`) |
| Motor GPIO | IN1–4: 18, 19, 32, 33 — see [docs/WIRING.md](docs/WIRING.md) |
| Encoder GPIO | LEFT **26, 27** (count negated); RIGHT **16, 17** |
| Encoder cross | **`BUDDY_L298_ENCODER_CROSS 0`** — direct map for TF/odom |
| Motor cross | **`BUDDY_L298_MOTOR_CROSS 1`** — L298 PWM crossed on chassis |
| Serial | **115200**, commands end with **CR** |
| Buddy device | `/dev/ttyUSB1` (lidar often `ttyUSB0`) |
| ROS motor mode | Open-loop PWM: serial **`o L R`**, buddy caps **130–230** |

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
| `description/ros2_control.xacro` | `device`, `baud_rate`, `enc_counts_per_rev`, `open_loop_min_pwm` (130), `open_loop_max_pwm` (230), `swap_motor_pwm` (false), `motor_scale` (1.0) |
| `description/drive_train.xacro` | `encoder_counts_per_rev`, wheel size |
| `config/controller.yaml` | `wheel_separation`, `wheel_radius` — normal `left_wheel_joint` / `right_wheel_joint` (no name swap) |

Nav2 also needs `twist_stamper` on the Pi (buddy launch) — not part of this repo.

After encoder or gear changes: update `encoder_counts_per_rev`, re-flash if GPIO or cross flags changed, **re-map** if odom changed.

## Test without ROS

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
| PWM below 130 no spin | Normal — buddy `open_loop_min_pwm` is **130**; use `test_motors.sh 130 …` |
| Auto-stop after 2 s | Normal without commands; buddy sends `o` at 30 Hz when driving |

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
