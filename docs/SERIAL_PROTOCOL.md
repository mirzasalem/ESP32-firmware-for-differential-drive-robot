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
| `m` | `m <L> <R>` + CR | Closed-loop: encoder ticks per PID frame (~30 Hz) |
| `o` | `o <L> <R>` + CR | Open-loop PWM −255…255 per side (buddy caps 130–230 on Pi) |
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

### Auto-stop

If no motor command arrives for **2 seconds** (`AUTO_STOP_INTERVAL`), PWM goes to 0.

### PID loop rate

**30 Hz** — `TargetTicksPerFrame` is encoder ticks **per frame**, not per second.

Approximate: `ticks_per_sec ≈ TargetTicksPerFrame × 30`.

Buddy default: **`use_open_loop_pwm: true`** → ROS sends `o` only. Set `use_open_loop_pwm: false` in `ros2_control.xacro` for closed-loop `m` + ESP PID.

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
