# Serial protocol (ROSArduinoBridge)

Used by `diffdrive_arduino` on the Pi and for bench testing.

| Parameter | Value |
|-----------|--------|
| Baud | **115200** |
| Line ending | **CR** only (`\r`, ASCII 13) |
| Arguments | Space-separated after command letter |

## Base commands (buddy uses these)

| Cmd | Format | Description |
|-----|--------|-------------|
| `e` | `e` + CR | Read encoders → `left right` + newline (see [WIRING.md](WIRING.md)) |
| `r` | `r` + CR | Reset encoders + PID → `OK` |
| `m` | `m <L> <R>` + CR | Closed-loop: encoder ticks per PID frame (~50 Hz). **No reply** |
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

Response: `OK <L> <R>` (echoes received values).

### `m` sends no reply

`m` arrives on every control frame and the Pi does not wait for an acknowledgement. Echoing `OK` per command filled the USB buffer and stalled the following `e` read, which showed up on the Pi as multi-hundred-millisecond `Read time` warnings, `Encoder read failed`, and Nav2 `Failed to make progress`. Do not add the reply back.

Commands that **do** reply: `c`, `w`, `x`, `r`, `u` → `OK`; `o` → `OK <L> <R>`; `b`, `e`, `a`, `d`, `p` → data; unknown → `Invalid Command`.

### Auto-stop

If no motor command arrives for **2 seconds** (`AUTO_STOP_INTERVAL`), PWM goes to 0.

The Pi driver only writes `m` when the tick pair changes, so it also resends an unchanged command every 500 ms (`motor_keepalive_ms`) to stay inside this window. Lowering `AUTO_STOP_INTERVAL` below that keep-alive will cut off a steady cruise.

### PID loop rate

**50 Hz** — `TargetTicksPerFrame` is encoder ticks **per frame**, not per second.

Approximate: `ticks_per_sec ≈ TargetTicksPerFrame × 50`.

Buddy default: **`use_open_loop_pwm: false`** → ROS sends `m` + ESP PID. Set `use_open_loop_pwm: true` in `ros2_control.xacro` for open-loop `o` only.

### Velocity PID and the PWM ramp

`diff_controller.h` runs a velocity PID ported from the ROS1 joey_v1 I2C firmware. PWM is **not** floored at a breakaway value:

- Output is `(Kp*err - Kd*d_input + ITerm) / Ko`, clamped to ±`MAX_PWM`.
- `ITerm` accumulates `Ki * err` each frame and is clamped to ±`MAX_PWM * Ko`.
- A wheel held back by load keeps accumulating, so PWM climbs until it breaks free or saturates.
- `ITerm` resets on `m 0 0` and on a direction reversal, so a stop or reversal never inherits accumulated push.

**Ki is the ramp rate.** At `Ko = 50`, a 1 tick/frame shortfall adds `Ki/Ko` PWM per frame, so `Ki = 100` climbs about 100 PWM per second. Tune via `u` (or `pid_i` in the Pi's `ros2_control.xacro`) without reflashing. Acceleration shaping belongs to `diff_drive_controller` on the Pi, not here.

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
~/esp/esp2ros2/scripts/test_motors.sh /dev/ttyUSB1 130 130 3
~/esp/esp2ros2/scripts/test_motors_diag.sh /dev/ttyUSB1
```

Held-open fd example:

```bash
exec 3<>/dev/ttyUSB1
stty -F /dev/ttyUSB1 115200 raw -echo
sleep 2
printf 'o 130 130\r' >&3
sleep 3
printf 'o 0 0\r' >&3
exec 3>&-
```

Encoder read:

```bash
python3 -c "import serial,time; s=serial.Serial('/dev/ttyUSB1',115200,timeout=1); time.sleep(2); s.write(b'e\r'); time.sleep(0.2); print(s.read(64))"
```

## ROS 2 side (`diffdrive_arduino`)

| ROS action | Serial |
|------------|--------|
| Wheel velocity commands | `o <pwm_l> <pwm_r>` (open loop) or `m …` (closed loop) |
| Encoder read | `e` → integrate in plugin → `/odom` |

Parameters: `buddy/description/ros2_control.xacro` — `device`, `baud_rate`, `enc_counts_per_rev`, `open_loop_min_pwm` (130), `open_loop_max_pwm` (230), `motor_scale`, `swap_motor_pwm` (false).

Buddy motor tuning: `buddy/docs/DRIVE_TRAIN.md`.

Only **one** process on the ESP serial port (close Serial Monitor / `screen` before `ros2 launch`).
