# Serial protocol (ROSArduinoBridge)

Used by `diffdrive_arduino` on the Pi and for bench testing.

**Buddy verified defaults:** `m` fire-and-forget (no `OK`), `BUDDY_LEFT_ENCODER_INVERT 1`, Pi `negate_left_encoder_odom: false`, `encoder_read_divisor: 2`, `motor_keepalive_ms: 500`. This combination passes bench test, left/right turns under body weight, and Nav2 navigation on the real robot.

**PID gains (2026-09-02, floor-verified):** `u 100:80:350:50` — raised from `100:40:150:50` in two passes (150→200 unloaded, then 200→350 with the robot's real weight on the wheels) to cut the fixed dead time before breakaway (see §"Velocity PID and the PWM ramp" below).

| Parameter | Value |
|-----------|--------|
| Baud | **115200** |
| Line ending | **CR** only (`\r`, ASCII 13) |
| Arguments | Space-separated after command letter |

## Base commands (buddy uses these)

| Cmd | Format | Description |
|-----|--------|-------------|
| `e` | `e` + CR | Read encoders → `left right` + newline (see [WIRING.md](WIRING.md)) |
| `f` | `f` + CR | Read encoders **and** IMU in one round trip → `left right gx gy gz ax ay az imu_ok` |
| `g` | `g` + CR | Read IMU only → `gx gy gz ax ay az imu_ok` |
| `i` | `i` + CR | Re-estimate gyro bias (stops motors, blocks ~0.5 s) → `OK` or `IMU FAIL` |
| `r` | `r` + CR | Reset encoders + PID → `OK` |
| `m` | `m <L> <R>` + CR | Closed-loop: encoder ticks per PID frame (**20 Hz** on Buddy). **No reply** |
| `o` | `o <L> <R>` + CR | Open-loop PWM −255…255 per side (buddy caps 130–230 on Pi) → `OK <L> <R>` |
| `u` | `u Kp:Kd:Ki:Ko` + CR | Update PID (order **P:D:I:Ko**) → `OK` |
| `b` | `b` + CR | Print baud rate |

### `o` and `m` — left/right argument order

- **First number** = command for ROS **left** wheel (`wheel_l_` / `left_wheel_joint`).
- **Second number** = command for ROS **right** wheel.

Always send ROS joint order on the serial line. Buddy firmware applies:

| Layer | Flag (default) | Effect |
|-------|----------------|--------|
| Encoders | `BUDDY_L298_ENCODER_CROSS 0` | `e` left/right = physical left/right encoders |
| Motors | `BUDDY_L298_MOTOR_CROSS 1` | `setMotorSpeeds()` crosses PWM to L298 |

The Pi **does not** swap serial arguments (`swap_motor_pwm: false` in buddy `ros2_control.xacro`).

`MOTOR_RAW_PWM` handler:

```cpp
setMotorSpeeds(arg1, arg2);  // arg1 = ROS left joint, arg2 = ROS right joint
```

`READ_ENCODERS` (`e`):

```cpp
readEncoderRosLeft();   // first number  = left_wheel_joint
readEncoderRosRight();  // second number = right_wheel_joint
```

Response on **`o` only:** `OK <L> <R>` (echoes received values). **`m` sends no reply.**

### `m` sends no reply

`m` arrives on every control frame and the Pi does not wait for an acknowledgement. Echoing `OK` per command filled the USB buffer and stalled the following `e` read, which showed up on the Pi as multi-hundred-millisecond `Read time` warnings, `Encoder read failed`, and Nav2 `Failed to make progress`. Do not add the reply back.

Commands that **do** reply: `c`, `w`, `x`, `r`, `u`, `i` → `OK`; `o` → `OK <L> <R>`; `b`, `e`, `f`, `g`, `a`, `d`, `p` → data; unknown → `Invalid Command`.

### Auto-stop

If no motor command arrives for **2 seconds** (`AUTO_STOP_INTERVAL`), PWM goes to 0.

The Pi driver only writes `m` when the tick pair changes, so it also resends an unchanged command every 500 ms (`motor_keepalive_ms`) to stay inside this window. Lowering `AUTO_STOP_INTERVAL` below that keep-alive will cut off a steady cruise.

### PID loop rate

**20 Hz** on Buddy — `TargetTicksPerFrame` is encoder ticks **per frame**, not per second. This must match the Pi `loop_rate` in `ros2_control.xacro` (also **20**).

Approximate: `ticks_per_sec ≈ TargetTicksPerFrame × PID_RATE`.

If `PID_RATE` is 50 while ROS still divides by 20, measured **linear** `odom/cmd` on `/diff_drive_controller/*` is about **2.3–2.5**. `compare_cmd_vel_odom.py` cannot detect this: it only looks at **angular.z**, and with `use_imu:=true` that yaw is the gyro.

Buddy default: **`use_open_loop_pwm: false`** → ROS sends `m` + ESP PID. Set `use_open_loop_pwm: true` in `ros2_control.xacro` for open-loop `o` only.

### Velocity PID and the PWM ramp

`diff_controller.h` runs a velocity PID ported from the ROS1 joey_v1 I2C firmware. PWM is **not** floored at a breakaway value:

- Output is `(Kp*err - Kd*d_input + ITerm) / Ko`, clamped to ±`MAX_PWM`.
- `ITerm` accumulates `Ki * err` each frame and is clamped to ±`MAX_PWM * Ko`.
- A wheel held back by load keeps accumulating, so PWM climbs until it breaks free or saturates.
- `ITerm` resets on `m 0 0` and on a direction reversal, so a stop or reversal never inherits accumulated push.
- **Cross-wheel reset (fixed 2026-09-01):** a direction reversal on *one* wheel used to reset only that wheel's `ITerm` — a turn (opposite-sign L/R targets) followed immediately by forward/reverse (same-sign targets) flips only one wheel's sign, so the other wheel kept the `ITerm` it wound up fighting the turn and slammed that leftover push into the new move the instant it started. `updatePID()` now resets **both** wheels' `ITerm` together whenever *either* one's target sign flips (see `diff_controller.h`), so no maneuver starts carrying push accumulated by a different one.
- **Fixed dead time from a stop (2026-09-02, two bench passes):** because `ITerm` starts at 0 every time a move follows `m 0 0`, PWM always ramps up from zero at `Ki/Ko` PWM per frame regardless of the commanded target. *Unloaded* bench (wheels lifted, `scripts/pwm_sweep.py`) found free-spin breakaway around PWM 65-70 and a ~1.0-1.1s dead time at the original `Ki=150` before *any* wheel motion, for targets from `m 1 1` through `m 10 10` alike (tracking over a 3 s window landed at a near-constant 60-68% of the commanded ticks in every case — consistent with a fixed dead time, not a target-dependent effect). This reads as "PWM too low to move, then it moves much more than expected" once the target catches up, and as motor noise while stalled below breakaway. Raising `Ki` to 200 (`Kd` to 60) shortened the dead time on that bench without introducing hunting, but was unverified under real load. *Loaded* bench (robot's real weight on the wheels, same script run spin-in-place so it doesn't drive off the bench) measured **actual breakaway around PWM 75-80** — lower than the ~120 previously assumed — and found `Ki=200` still left `m 1 1` at only 38-48% and `m 2 2` at 55-57% over 3s. Raised again to **`Ki=350`, `Kd=80`**: the same loaded test improved `m 1 1` to ~63% and `m 2 2` to ~59-65%, with `m 4` through `m 10` essentially unchanged (~65-70%, since they were never dead-time-starved) and no divergence between left/right.

**Stopping behaviour (measured 2026-09-07).** `m 0 0` calls `setMotorSpeeds(0, 0)`, which drives both L298 inputs low with `EN` high — that is a **brake** (motor terminals shorted), not a coast. It still does not stop the robot dead: momentum through the gearbox carries it roughly **0.21 m** from 0.30 m/s, and a spin carries **~24° at 0.85 rad/s, ~49° at 1.6 rad/s**. Feeding a decelerating ramp of `m` targets instead of a single `m 0 0` roughly halves that (49° → 25° at 1.6 rad/s), because the PID keeps actively braking toward each new lower target.

Two consequences for the Pi side, both handled in `buddy/config/`:

- Effective angular deceleration tops out near **2.9 rad/s²**, well under the ~1.2 m/s² the wheels manage linearly. Any nav2 parameter that *predicts* braking distance (notably `behavior_server` Spin's `rotational_acc_lim`, which computes `sqrt(2 · acc_lim · remaining)`) must be set at or below what the chassis actually achieves, or it brakes too late and overshoots.
- There is an irreducible **~6–9°** of rotation after the command reaches zero, regardless of how hard the ramp is, so chasing ever-higher deceleration limits stops paying off quickly.

**Ki is the ramp rate.** At `Ko = 50`, a 1 tick/frame shortfall adds `Ki/Ko` PWM per frame, so **`Ki = 350` (current default) at 20 Hz climbs about 140 PWM per second** (was 80 PWM/s at `Ki=200`, 60 PWM/s at the original `Ki=150`). Tune via `u` (or `pid_i` in the Pi's `ros2_control.xacro`) without reflashing. Acceleration shaping belongs to `diff_drive_controller` on the Pi, not here.

### Default PID gains (buddy)

| Gain | Firmware default | Buddy `ros2_control.xacro` | Role |
|------|------------------|----------------------------|------|
| Kp | 100 | `pid_p` | Tracking — proportional to tick error |
| Kd | **80** | `pid_d` | Damping — reduces overshoot after breakaway (raised 2026-09-02, floor-verified) |
| Ki | **350** | `pid_i` | **Ramp / breakaway** — pushes harder while stalled (raised 2026-09-02 to shorten dead time; was 150, then 200 unloaded) |
| Ko | 50 | `pid_o` | Output scaling divisor |

Send: `u 100:80:350:50` + CR → `OK`.

## IMU (`f`, `g`, `i`)

Optional MPU9250 on I2C — GPIO 21 (SDA) / 22 (SCL), 3.3 V. Enabled by `#define USE_IMU` in `ROSArduinoBridge.ino` (default **on**); comment it out and the sketch behaves exactly as before, with `f` reporting `imu_ok 0`. Wiring: [WIRING.md](WIRING.md).

**Units are milli-units, as integers:** gyro **mrad/s**, accel **mm/s²**. Integers keep the line short and parsing free of float formatting surprises. Divide by 1000 for rad/s and m/s².

```
f\r  ->  1234 -1187 2 -1 118 -31 44 9805 1
         |    |     |       |          |
         encL encR  gyro    accel      imu_ok
```

Above, `gz = 118 mrad/s` (turning left at ~6.8 °/s) and `az ≈ 9805 mm/s²` (gravity, board flat and upright). A still robot should read all three gyro axes within a few mrad/s of zero.

**Why `f` exists.** A USB round trip costs more than one control frame, which is why the Pi already decimates `e` to 10 Hz. Polling `e` and `g` separately would double that cost, so `f` returns both and the IMU is free in serial terms.

**Chip frame, not robot frame.** The reply is raw MPU9250 axes in SI units. Mount orientation is described once, by the `imu_link` origin in `buddy/description/imu.xacro` — never by flipping signs here. The magnetometer is not read at all: next to the motors, the L298 and the chassis metal, a compass heading is worse than none.

**Bias.** `imuInit()` averages 200 samples at boot, so the robot must be **still at power-on**. Send `i` to re-estimate later; the Pi driver does this automatically on activate (`imu_calibrate_on_activate`), which also covers the temperature drift since the ESP was plugged in.

### Bench scripts

```bash
~/esp/esp2ros2/scripts/test_closed_loop.sh /dev/ttyACM0 3   # closed-loop m (recommended)
~/esp/esp2ros2/scripts/test_motors.sh /dev/ttyACM0 130 130 2  # open-loop o
~/esp/esp2ros2/scripts/test_motors_diag.sh /dev/ttyACM0
python3 ~/esp/esp2ros2/scripts/pwm_sweep.py /dev/ttyACM0    # breakaway PWM + low-target tracking sweep
```

`test_closed_loop.sh` strips `\r` from encoder lines (ESP `Serial.println` uses `\r\n`). Do not wait for a reply after `m`.

## Other commands (optional)

| Cmd | Format | Description |
|-----|--------|-------------|
| `a` | `a <pin>` | Analog read |
| `d` | `d <pin>` | Digital read |
| `w` | `w <pin> <0\|1>` | Digital write |
| `x` | `x <pin> <val>` | Analog write |
| `c` | `c <pin> <0\|1>` | pinMode |
| `p` | `p <pin>` | Ultrasonic ping (if wired) |

Servo commands (`s`, `t`) only if `USE_SERVOS` is enabled in `ROSArduinoBridge.ino` (default **off**).

## Examples (shell)

**Do not** use `printf 'o 80 80\r' > /dev/ttyUSB1` alone — each open may **reset USB** and drop the command. Use:

```bash
~/esp/esp2ros2/scripts/test_motors.sh /dev/ttyACM0 130 130 3
~/esp/esp2ros2/scripts/test_motors_diag.sh /dev/ttyACM0
~/esp/esp2ros2/scripts/test_closed_loop.sh /dev/ttyACM0 3
```

Held-open fd example:

```bash
exec 3<>/dev/ttyACM0
stty -F /dev/ttyACM0 115200 raw -echo
sleep 2
printf 'm 5 5\r' >&3    # no reply
sleep 2
printf 'e\r' >&3
timeout 0.5 cat <&3
printf 'm 0 0\r' >&3
exec 3>&-
```

Encoder read:

```bash
python3 -c "import serial,time; s=serial.Serial('/dev/ttyACM0',115200,timeout=1); time.sleep(2); s.write(b'e\r'); time.sleep(0.2); print(s.read(64))"
```

## ROS 2 side (`diffdrive_arduino`)

| ROS action | Serial |
|------------|--------|
| Wheel velocity (closed-loop default) | `m <tick_l> <tick_r>` — write-on-change + 500 ms keep-alive |
| Wheel velocity (open-loop fallback) | `o <pwm_l> <pwm_r>` |
| Encoder read | `e` every Nth frame (`encoder_read_divisor: 2` → 10 Hz) → `/odom` |
| Encoder + IMU read (`enable_imu: true`) | `f` at the same 10 Hz → `/odom` + `/imu_sensor_broadcaster/imu` |
| Activate | `u P:D:I:Ko`, `i` if `imu_calibrate_on_activate`, then `m 0 0` on deactivate |

Key parameters in `buddy/description/ros2_control.xacro`:

| Parameter | Default | Notes |
|-----------|---------|-------|
| `use_open_loop_pwm` | false | Closed-loop `m` |
| `loop_rate` | 20 | Hz — matches `controller.yaml` |
| `encoder_read_divisor` | 2 | Poll `e` every 2nd frame |
| `motor_keepalive_ms` | 500 | Resend unchanged `m` (ESP auto-stop 2 s) |
| `pid_p/d/i/o` | 100/80/350/50 | Sent on activate (raised 2026-09-02, floor-verified with real load) |
| `enable_imu` | false | `true` → poll `f` instead of `e`; set from `use_imu:=true` at launch |
| `imu_calibrate_on_activate` | true | Send `i` on activate (no effect unless `enable_imu`) |
| `open_loop_min_pwm` / `max` | 130 / 230 | Open-loop path only |
| `swap_motor_pwm` | false | Do not double-cross with firmware `MOTOR_CROSS` |

Closed-loop driver: plain rad/s → ticks conversion — **no** tick slew, spin boost, or command filter.

Buddy motor tuning: [buddy/docs/DRIVE_TRAIN.md](https://github.com/mirzasalem/buddy-ros2/blob/main/docs/DRIVE_TRAIN.md) §3.

Only **one** process on the ESP serial port (close Serial Monitor / `screen` before `ros2 launch`).
