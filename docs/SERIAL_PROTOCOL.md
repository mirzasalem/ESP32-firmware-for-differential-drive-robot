# Serial Protocol Reference

USB CDC text protocol between the ESP32 firmware and a host (`diffdrive_arduino` / bench tools).

| Parameter | Value |
|-----------|--------|
| Baud rate | **115200** |
| Line ending | **CR** only (`\r`, ASCII 13) |
| Argument format | Space-separated after the command letter |

**Recommended defaults (Buddy, verified):** closed-loop `m` (no acknowledgement), PID `u 100:80:350:50`, `BUDDY_LEFT_ENCODER_INVERT 1`, host `negate_left_encoder_odom: false`, `encoder_read_divisor: 2`, `motor_keepalive_ms: 500`. This combination passes the bench test, left/right turns under body weight, and Nav2 navigation on the real robot.

**PID gains (2026-09-02, floor-verified):** `u 100:80:350:50`, raised from `100:40:150:50` in two passes (Ki 150→200 unloaded, then 200→350 with the robot's real weight on the wheels) to cut the fixed dead time before breakaway. See [Velocity PID](#velocity-pid) below.

---

## End-to-end data path

```text
                    ┌─────────────────────────────────────┐
                    │  ESP32 ROSArduinoBridge             │
                    │  velocity PID          20 Hz        │
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
| `m` | `m <L> <R>` + CR | Closed-loop ticks per PID frame (**20 Hz** on Buddy) — **no reply** |
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

`m` is issued every control frame. Acknowledging each command fills the USB buffer and delays the next encoder/IMU read. On the host that showed up as multi-hundred-millisecond `Read time` warnings, `Encoder read failed`, and Nav2 `Failed to make progress`. Do not add the reply back.

Commands that reply: `c`, `w`, `x`, `r`, `u`, `i` → `OK`; `o` → `OK <L> <R>`; `b`, `e`, `f`, `g`, `a`, `d`, `p` → data. Unknown commands return `Invalid Command`.

### Auto-stop

If no motor command arrives for **2 s** (`AUTO_STOP_INTERVAL`), PWM is set to zero. The host should resend an unchanged `m` at least every **500 ms** during steady cruise.

### Velocity PID

Loop rate: **20 Hz** on Buddy (`PID_RATE` in `ROSArduinoBridge.ino`). `TargetTicksPerFrame` is ticks **per frame** (not per second), and the rate must match the host `loop_rate` in `ros2_control.xacro` (also **20**):

```text
ticks_per_sec ≈ TargetTicksPerFrame × PID_RATE
```

If `PID_RATE` is 50 while the host still divides by 20, the measured **linear** `odom/cmd` ratio on `/diff_drive_controller/*` is about **2.3–2.5**, i.e. the robot overspeeds. `compare_cmd_vel_odom.py` cannot detect this: it only looks at **angular.z**, and with `use_imu:=true` that yaw comes from the gyro.

Output: `(Kp·err − Kd·d_input + ITerm) / Ko`, clamped to ±`MAX_PWM`.

- `ITerm` accumulates `Ki · err` each frame, clamped to ±`MAX_PWM · Ko`. A wheel held back by load keeps accumulating, so PWM climbs until it breaks free or saturates.
- `ITerm` resets on `m 0 0` and on direction reversal.
- **Cross-wheel reset (fixed 2026-09-01):** a reversal on *one* wheel used to reset only that wheel's `ITerm`. A turn (opposite-sign targets) followed immediately by forward/reverse (same-sign targets) flips only one wheel's sign, so the other wheel kept the `ITerm` it wound up during the turn and slammed it into the new move, felt as an asymmetric lurch. `updatePID()` now resets **both** wheels' `ITerm` whenever *either* target sign flips.
- **Fixed dead time from a stop (2026-09-02, two bench passes):** `ITerm` starts at 0 after `m 0 0`, so PWM always ramps up from zero at `Ki/Ko` PWM per frame, whatever the target. *Unloaded* bench (wheels lifted, `scripts/pwm_sweep.py`): free-spin breakaway around PWM 65–70, and ~1.0–1.1 s of dead time at the original `Ki=150` before any motion, for `m 1 1` through `m 10 10` alike (tracking over 3 s landed at a near-constant 60–68 % of the commanded ticks). That feels like "PWM too low to move, then it moves much more than expected", with motor noise while stalled. `Ki=200` / `Kd=60` shortened it on that bench. *Loaded* bench (the robot's real weight, spin-in-place): **real breakaway around PWM 75–80**, and `Ki=200` still left `m 1 1` at 38–48 % and `m 2 2` at 55–57 %. At **`Ki=350`, `Kd=80`**, `m 1 1` improved to ~63 % and `m 2 2` to ~59–65 %, with `m 4`…`m 10` unchanged (~65–70 %) and no left/right divergence.
- Acceleration shaping belongs on the host `diff_drive_controller`, not in this loop.

**Ki is the ramp rate.** At `Ko = 50`, a 1 tick/frame shortfall adds `Ki/Ko` PWM per frame, so **`Ki = 350` at 20 Hz climbs about 140 PWM per second** (80 PWM/s at `Ki=200`, 60 PWM/s at the original `Ki=150`). Tune via `u` (or `pid_i` in the host's `ros2_control.xacro`) without reflashing.

| Gain | Firmware default | Buddy `ros2_control.xacro` | Role |
|------|------------------|----------------------------|------|
| Kp | 100 | `pid_p` | Tracking, proportional to tick error |
| Kd | **80** | `pid_d` | Damping after breakaway (raised 2026-09-02, floor-verified) |
| Ki | **350** | `pid_i` | **Ramp / breakaway**, pushes harder while stalled (was 150, then 200 unloaded) |
| Ko | 50 | `pid_o` | Output scaling divisor |

Example: `u 100:80:350:50\r` → `OK`. The firmware values are only the bench fallback. The host sends its `pid_*` on activate, so keep both in sync.

### Stopping behaviour (measured 2026-09-07)

`m 0 0` calls `setMotorSpeeds(0, 0)`, which drives both L298 inputs low with `EN` high. That is a **brake** (motor terminals shorted), not a coast. It still doesn't stop the robot dead: momentum through the gearbox carries it roughly **0.21 m** from 0.30 m/s, and a spin carries **~24° at 0.85 rad/s, ~49° at 1.6 rad/s**. Feeding a decelerating ramp of `m` targets instead of a single `m 0 0` roughly halves that (49° → 25° at 1.6 rad/s), because the PID keeps actively braking toward each lower target.

Two consequences for the host, both handled in `buddy/config/`:

- Effective angular deceleration tops out near **2.9 rad/s²**, well under the ~1.2 m/s² the wheels manage linearly. Any nav2 parameter that *predicts* braking distance (notably `behavior_server` Spin's `rotational_acc_lim`, which computes `sqrt(2 · acc_lim · remaining)`) must be at or below what the chassis actually achieves, or it brakes too late and overshoots.
- There is an irreducible **~6–9°** of rotation after the command reaches zero, however hard the ramp, so chasing ever-higher deceleration limits stops paying off quickly.

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

In the example above, `gz = 118 mrad/s` (turning left at ~6.8 °/s) and `az ≈ 9805 mm/s²` (gravity, board flat and upright). A still robot reads all three gyro axes within a few mrad/s of zero.

The host already decimates `e` to 10 Hz because a USB round trip costs more than one control frame, which is why `f` returns both. Mount orientation is described once, by the `imu_link` origin (buddy `description/imu.xacro`, fed by `chassis_imu_*` in `chassis.xacro`). The host driver sends `i` automatically on activate (`imu_calibrate_on_activate`), which also covers the temperature drift since the ESP was plugged in.

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
./scripts/test_closed_loop.sh /dev/ttyACM0 3        # closed-loop m (recommended)
./scripts/test_motors.sh /dev/ttyACM0 130 130 2     # open-loop o
./scripts/test_motors_diag.sh /dev/ttyACM0
python3 ./scripts/pwm_sweep.py /dev/ttyACM0         # breakaway PWM + low-target tracking sweep
python3 ./scripts/calibrate_360.py /dev/ttyACM0     # gyro-referenced encoder/track check (USE_IMU)
```

`test_closed_loop.sh` strips `\r` from encoder lines (ESP `Serial.println` uses `\r\n`). Don't wait for a reply after `m`. Don't `printf 'o 80 80\r' > /dev/ttyACM0` on its own either: each open may reset USB and drop the command.

Encoder read:

```bash
python3 -c "import serial,time; s=serial.Serial('/dev/ttyACM0',115200,timeout=1); time.sleep(2); s.write(b'e\r'); time.sleep(0.2); print(s.read(64))"
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
| Closed-loop velocity | `m <tick_l> <tick_r>` (write-on-change + 500 ms keep-alive) |
| Open-loop velocity | `o <pwm_l> <pwm_r>` |
| Encoders only | `e` every Nth frame (`encoder_read_divisor: 2` → 10 Hz) → `/odom` |
| Encoders + IMU | `f` at the same 10 Hz → `/odom` + `/imu_sensor_broadcaster/imu` |
| Activate | `u P:D:I:Ko`, `i` if `imu_calibrate_on_activate`, then run; `m 0 0` on deactivate |

Parameters in `buddy/description/ros2_control.xacro`:

| Parameter | Typical default | Notes |
|-----------|-----------------|-------|
| `use_open_loop_pwm` | `false` | Closed-loop `m` |
| `loop_rate` | `20` Hz | Must match firmware `PID_RATE` and `controller.yaml` |
| `encoder_read_divisor` | `2` (~10 Hz polls) | |
| `motor_keepalive_ms` | `500` | Resend unchanged `m` (ESP auto-stop is 2 s) |
| `pid_p/d/i/o` | `100/80/350/50` | Sent on activate (raised 2026-09-02, floor-verified with real load) |
| `enable_imu` | `false` | `true` → poll `f` instead of `e`; set from `use_imu:=true` at launch |
| `imu_calibrate_on_activate` | `true` | Send `i` on activate (no effect unless `enable_imu`) |
| `open_loop_min_pwm` / `max` | `130` / `230` | Open-loop path only |
| `swap_motor_pwm` | `false` | Don't double-cross with firmware `MOTOR_CROSS` |

The closed-loop driver is a plain rad/s → ticks conversion, with **no** tick slew, spin boost, or command filter. Buddy motor tuning: [buddy/docs/DRIVE_TRAIN.md](https://github.com/mirzasalem/buddy-ros2/blob/main/docs/DRIVE_TRAIN.md) §3.

Only **one** process may own the ESP32 serial port.
