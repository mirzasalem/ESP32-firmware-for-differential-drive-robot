# Serial Protocol Reference

USB CDC text protocol between the ESP32 firmware and a host (`diffdrive_arduino` / bench tools).

| Parameter | Value |
|-----------|--------|
| Baud rate | **115200** |
| Line ending | **CR** only (`\r`, ASCII 13) |
| Argument format | Space-separated after the command letter |

**Recommended defaults:** closed-loop `m` (no acknowledgement), PID `u 100:40:150:50`, encoder invert on left channel, host `encoder_read_divisor: 2`, `motor_keepalive_ms: 500`.

---

## End-to-end data path

```text
                    ┌─────────────────────────────────────┐
                    │  ESP32 ROSArduinoBridge             │
                    │  velocity PID          50 Hz        │
                    │  IMU sample            100 Hz       │
                    └──────────────┬──────────────────────┘
                                   │ USB 115200, CR
           m L R                   │           f or e
           u P:D:I:Ko              │
           i (gyro bias)           │
                                   ▼
                    ┌─────────────────────────────────────┐
                    │  Host: diffdrive_arduino            │
                    │  (ros2_control hardware interface)  │
                    └──────────────┬──────────────────────┘
                                   │
              IMU off              │              IMU on
              poll e               │              poll f
                    │              │              │
                    ▼              │              ▼
         diff_drive_controller     │    + imu_sensor_broadcaster
                    │              │              │
                    │              │              ▼
                    │              │       ekf_filter_node
                    │              │       (vx + vyaw)
                    ▼              │              ▼
              odom relay → /odom   │       fused /odom + TF
```

| Mode | ESP poll | `/odom` publisher |
|------|----------|-------------------|
| Encoders only | `e` | Relay from `diff_drive_controller` |
| Encoders + IMU | `f` | EKF (`robot_localization`) |

---

## Command set

| Cmd | Format | Description |
|-----|--------|-------------|
| `e` | `e` + CR | Read encoders → `left right` |
| `f` | `f` + CR | Read encoders and IMU → `left right gx gy gz ax ay az imu_ok` |
| `g` | `g` + CR | Read IMU → `gx gy gz ax ay az imu_ok` |
| `i` | `i` + CR | Recalibrate gyro bias (~0.5 s, robot still) → `OK` or `IMU FAIL` |
| `r` | `r` + CR | Reset encoders and PID → `OK` |
| `m` | `m <L> <R>` + CR | Closed-loop ticks per 50 Hz frame — **no reply** |
| `o` | `o <L> <R>` + CR | Open-loop PWM (−255…255) → `OK <L> <R>` |
| `u` | `u Kp:Kd:Ki:Ko` + CR | Update PID gains → `OK` |
| `b` | `b` + CR | Print baud rate |

### Argument order for `m` and `o`

1. First value = ROS **left** wheel joint  
2. Second value = ROS **right** wheel joint  

Firmware applies mapping flags from `buddy_robot_config.h`:

| Layer | Default | Effect |
|-------|---------|--------|
| Encoders | `ENCODER_CROSS 0` | `e`/`f` left/right = physical left/right |
| Motors | `MOTOR_CROSS 1` | PWM crossed in `setMotorSpeeds()` |

The host must not also swap serial arguments (`swap_motor_pwm: false`).

### Why `m` has no reply

`m` is issued every control frame. Acknowledging each command fills the USB buffer and delays the next encoder/IMU read. Commands that reply: `r`, `u`, `i`, `o`, `b`, `e`, `f`, `g`, and GPIO helpers. Unknown commands return `Invalid Command`.

### Auto-stop

If no motor command arrives for **2 s** (`AUTO_STOP_INTERVAL`), PWM is set to zero. The host should resend an unchanged `m` at least every **500 ms** during steady cruise.

### Velocity PID

Loop rate: **50 Hz**. `TargetTicksPerFrame` is ticks **per frame** (not per second):

```text
ticks_per_sec ≈ TargetTicksPerFrame × 50
```

Output: `(Kp·err − Kd·d_input + ITerm) / Ko`, clamped to ±`MAX_PWM`.

- `ITerm` accumulates under load (breakaway / ramp behaviour).
- `ITerm` resets on `m 0 0` and on direction reversal.
- Acceleration shaping belongs on the host `diff_drive_controller`, not in this loop.

| Gain | Default | Role |
|------|---------|------|
| Kp | 100 | Tracking |
| Kd | 40 | Damping |
| Ki | 150 | Integral ramp under load |
| Ko | 50 | Output scaling |

Example: `u 100:40:150:50\r` → `OK`.

---

## IMU commands (`f`, `g`, `i`)

Optional MPU9250 on I2C (SDA **21**, SCL **22**, 3.3 V). Enabled by `#define USE_IMU`. Wiring: [WIRING.md](WIRING.md).

**Units:** integer milli-units — gyro in **mrad/s**, accel in **mm/s²**. Divide by 1000 for SI.

```text
f\r  →  1234 -1187  2 -1 118  -31 44 9805  1
        encL  encR  ----gyro----  --accel--  ok
```

| Design choice | Rationale |
|---------------|-----------|
| Combined `f` poll | Avoids a second USB round-trip versus separate `e` + `g` |
| Chip frame axes | Mount orientation belongs in URDF, not firmware sign flips |
| No magnetometer | Indoor magnetic field near motors/H-bridge is unreliable |
| Boot bias average | Robot must be still at power-on; `i` re-estimates later |

| API | Location | Role |
|-----|----------|------|
| `imuInit()` | `imu_driver.ino` | Probe, configure ±250 °/s / ±2 g, bias |
| `imuUpdate()` | `imu_driver.ino` | 100 Hz sample |
| `imuPrintReading()` | `imu_driver.ino` | Serial milli-unit output for `f`/`g` |

---

## Optional GPIO / sensor commands

| Cmd | Format | Description |
|-----|--------|-------------|
| `a` | `a <pin>` | Analog read |
| `d` | `d <pin>` | Digital read |
| `w` | `w <pin> <0\|1>` | Digital write |
| `x` | `x <pin> <val>` | Analog write |
| `c` | `c <pin> <0\|1>` | `pinMode` |
| `p` | `p <pin>` | Ultrasonic ping (if wired) |

Servo commands (`s`, `t`) require `#define USE_SERVOS` (default off).

---

## Bench examples

Prefer the provided scripts (opening `/dev/tty*` briefly can reset USB CDC):

```bash
./scripts/test_closed_loop.sh /dev/ttyACM0 3
./scripts/test_motors.sh /dev/ttyACM0 130 130 2
./scripts/test_motors_diag.sh /dev/ttyACM0
```

Held-open file descriptor:

```bash
exec 3<>/dev/ttyACM0
stty -F /dev/ttyACM0 115200 raw -echo
sleep 2
printf 'm 5 5\r' >&3
sleep 2
printf 'e\r' >&3
timeout 0.5 cat <&3
printf 'm 0 0\r' >&3
exec 3>&-
```

---

## Host (`diffdrive_arduino`) mapping

| Host action | Serial |
|-------------|--------|
| Closed-loop velocity | `m <tick_l> <tick_r>` (write-on-change + keep-alive) |
| Open-loop velocity | `o <pwm_l> <pwm_r>` |
| Encoders only | `e` at reduced rate |
| Encoders + IMU | `f` at the same rate |
| Activate | `u …`, optional `i`, then run; `m 0 0` on deactivate |

| Parameter | Typical default |
|-----------|-----------------|
| `use_open_loop_pwm` | `false` |
| `loop_rate` | `20` Hz |
| `encoder_read_divisor` | `2` (~10 Hz polls) |
| `motor_keepalive_ms` | `500` |
| `pid_p/d/i/o` | `100/40/150/50` |
| `enable_imu` | `false` (set `true` with `use_imu:=true`) |
| `swap_motor_pwm` | `false` |

Only **one** process may own the ESP32 serial port.
