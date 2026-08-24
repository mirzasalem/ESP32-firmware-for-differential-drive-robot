# Wiring Reference

Pin assignments for ESP32 + L298N + quadrature encoders + optional MPU9250.

Source of truth in firmware: `motor_driver.h`, `encoder_driver.h`, `imu_driver.h`, `buddy_robot_config.h`.

**Default mapping profile**

| Flag | Value | Effect |
|------|-------|--------|
| `BUDDY_L298_ENCODER_CROSS` | `0` | Encoder left/right match physical sides |
| `BUDDY_L298_MOTOR_CROSS` | `1` | Motor PWM crossed to match chassis wiring |
| `BUDDY_LEFT_ENCODER_INVERT` | `1` | Left encoder sign matches forward motion |

On the host, keep `swap_motor_pwm: false` and `negate_left_encoder_odom: false` when using these defaults.

---

## Power

```text
Motor battery (7–12 V) ──► L298N +12V / VMS
                       └──► motor outputs

ESP32 USB ──► PC or Raspberry Pi (logic + serial only)

Common ground (required):
  ESP32 GND  ↔  L298N GND  ↔  battery GND
```

| Rule | Detail |
|------|--------|
| Motor power | From the battery through the L298N only |
| Logic power | ESP32 from USB — do **not** power motors from ESP32 3.3 V / 5 V |
| Ground | Shared ground between ESP32, L298N, and battery is mandatory |

---

## L298N → ESP32

| L298N label | ESP32 GPIO | Role |
|-------------|------------|------|
| IN1 | **18** | Left channel direction A |
| IN2 | **19** | Left channel direction B |
| IN3 | **32** | Right channel direction A |
| IN4 | **33** | Right channel direction B |
| ENA | **25** or jumper to 5 V | Left enable |
| ENB | **14** or jumper to 5 V | Right enable |
| GND | GND | Common ground |

Many L298N modules ship with **ENA/ENB jumpers** installed (always enabled). In that case only IN1–IN4 need GPIO wires.

### Motor outputs

| L298N outputs | Physical wheel |
|---------------|----------------|
| OUT1, OUT2 | Left |
| OUT3, OUT4 | Right |

Encoders should be mounted on the same side as each wheel (left encoder on GPIO 26/27, right on 16/17).

### Motor / encoder cross mapping

On some chassis builds, L298 motor leads are crossed relative to encoder sides. This firmware uses two independent flags:

| Flag | Default | Effect |
|------|---------|--------|
| `BUDDY_L298_ENCODER_CROSS` | `0` | Serial/`e` left = physical left encoder |
| `BUDDY_L298_MOTOR_CROSS` | `1` | `setMotorSpeeds()` crosses PWM to the L298 |

| ROS / serial joint | Encoder (`ENCODER_CROSS=0`) | Motor PWM (`MOTOR_CROSS=1`) |
|--------------------|-----------------------------|-----------------------------|
| Left (`arg1`) | GPIO 26, 27 (physical left) | OUT3/4 |
| Right (`arg2`) | GPIO 16, 17 (physical right) | OUT1/2 |

Do **not** also swap wheel names or set `swap_motor_pwm: true` on the host — that double-swaps turn direction.

If a single wheel runs backward for forward commands, reverse that motor’s two leads, or invert `left_motor_scale` / `right_motor_scale` on the host.

---

## Encoders → ESP32

| Label | GPIO A | GPIO B | Side | Notes |
|-------|--------|--------|------|-------|
| LEFT | **26** | **27** | Left wheel | `BUDDY_LEFT_ENCODER_INVERT 1` |
| RIGHT | **16** | **17** | Right wheel | Counts used as-is |

- Use **3.3 V** logic-level encoders.
- Firmware enables internal weak pull-ups via ESP32Encoder.
- After changing pins, re-upload firmware and update `encoder_counts_per_rev` on the host.

---

## MPU9250 IMU → ESP32 (I2C)

Enabled with `#define USE_IMU` in `ROSArduinoBridge.ino`.

| MPU9250 | ESP32 | Notes |
|---------|-------|-------|
| VCC | **3.3 V** | Prefer 3.3 V for correct SDA/SCL levels |
| GND | GND | Shared with ESP32 / L298 / battery |
| SDA | **21** | |
| SCL | **22** | |
| AD0 | GND or open | Address `0x68` (or `0x69` if tied high; both probed) |
| NCS, FSYNC, INT | Unconnected | Polled at 100 Hz; interrupts unused |

Four wires are sufficient. The magnetometer is not initialized or read.

### Mounting

1. **Rigid mount** — avoid vibration from motors or the L298 heatsink.
2. **Known orientation** — define mount rotation in the host URDF (`imu_link`), not by flipping signs in firmware.

Keep the robot **stationary at power-on** so boot-time gyro bias estimation is valid. Recalibrate later with serial command `i`.

---

## USB device ports (typical)

| Device | Typical port |
|--------|----------------|
| ESP32 | `/dev/ttyACM0`, `/dev/ttyACM1`, or stable by-id |
| RPLIDAR (if present) | `/dev/ttyUSB0` |

Only one process may open the ESP32 serial port at a time.

---

## System diagram

```text
                    ┌─────────────┐
   Motor battery ──►│   L298N     ├──► OUT1/2  → left wheel
                    │  IN1..IN4   ├──► OUT3/4  → right wheel
                    └──────┬──────┘
                           │ GPIO 18, 19, 32, 33
                    ┌──────▼──────┐
   Enc L: 26, 27 ──►│   ESP32     │◄── USB ──► Host (ROS 2)
   Enc R: 16, 17 ──►│             │
   MPU9250 ────────►│ SDA 21      │   I2C 0x68, 3.3 V (optional)
        (4 wires)   │ SCL 22      │
                    └─────────────┘
```

---

## Changing pins or mapping flags

1. Edit `buddy_robot_config.h`, `motor_driver.h`, and/or `encoder_driver.h`.
2. Re-upload `firmware/ROSArduinoBridge/ROSArduinoBridge.ino`.
3. Update host parameters (`enc_counts_per_rev`, `device`, motor scales) only as needed.

---

## Bench checks

### Open-loop motors

```bash
./scripts/test_motors_diag.sh /dev/ttyACM0
./scripts/test_motors.sh /dev/ttyACM0 130 130 2
```

| Command | Meaning (default mapping) |
|---------|---------------------------|
| `o 130 130` | Both joints forward |
| `o 130 0` | Left joint command |
| `o 0 130` | Right joint command |
| `o 130 -130` | In-place spin |

### Closed-loop velocity PID

```bash
./scripts/test_closed_loop.sh /dev/ttyACM0 2
```

| Command | Meaning |
|---------|---------|
| `m 5 5` | Both wheels forward (~250 ticks/s at steady state) |
| `m 4 -4` | In-place spin |
| `m 0 0` | Stop and reset PID integral |

### IMU

With the robot still:

```text
g\r  →  1 -2 0  -35 51 9803  1
```

Pass criteria: trailing `imu_ok = 1`, gyro near zero, `az ≈ 9800` mm/s² when the board is flat and upright.

---

## Related documents

- [ARDUINO_SETUP.md](ARDUINO_SETUP.md) — firmware upload
- [SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md) — serial commands
- [../README.md](../README.md) — project overview
