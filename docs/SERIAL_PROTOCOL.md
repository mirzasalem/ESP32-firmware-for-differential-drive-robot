# Serial protocol (ROSArduinoBridge)

Used by `diffdrive_arduino` on the Pi and for bench testing.

**Buddy verified defaults:** `m` fire-and-forget (no `OK`), `u 100:40:150:50`, `BUDDY_LEFT_ENCODER_INVERT 1`, Pi `negate_left_encoder_odom: false`, `encoder_read_divisor: 2`, `motor_keepalive_ms: 500`. This combination passes bench test, left/right turns under body weight, and Nav2 navigation on the real robot.

| Parameter | Value |
|-----------|--------|
| Baud | **115200** |
| Line ending | **CR** only (`\r`, ASCII 13) |
| Arguments | Space-separated after command letter |

## End-to-end integration (motors + IMU)

```text
                    ┌─────────────────────────────────────┐
                    │  ESP32 ROSArduinoBridge             │
                    │  diff_controller.h  (50 Hz PID)     │
                    │  imu_driver.ino     (100 Hz, I2C)   │
                    └──────────────┬──────────────────────┘
                                   │ USB 115200 CR
           m L R (20 Hz)           │           f or e (10 Hz)
           u P:D:I:Ko             │
           i (bias, on activate)   │
                                   ▼
                    ┌─────────────────────────────────────┐
                    │  diffdrive_arduino (ros2_control)   │
                    └──────────────┬──────────────────────┘
                                   │
              use_imu:=false       │       use_imu:=true
              poll "e"             │       poll "f"
                    │              │              │
                    ▼              │              ▼
         diff_drive_controller     │    + imu_sensor_broadcaster
         /diff_drive_controller/odom              │
                    │              │              ▼
                    │              │       ekf_filter_node
                    │              │       (vx wheels + vyaw gyro)
                    ▼              │              ▼
              odom_relay ──► /odom │       /odom (fused) + TF
```

| Mode | ESP poll | Buddy nodes | `/odom` publisher |
|------|----------|-------------|-------------------|
| Default | **`e`** | `odom_relay` | relay from `/diff_drive_controller/odom` |
| IMU | **`f`** | `imu_sensor_broadcaster`, `ekf_filter_node` | EKF (remapped from `odometry/filtered`) |

Firmware source: [../README.md#firmware-source-code](../README.md#firmware-source-code). Run commands: [../README.md#run-on-the-robot-with-imu](../README.md#run-on-the-robot-with-imu).

## Base commands (buddy uses these)

| Cmd | Format | Description |
|-----|--------|-------------|
| `e` | `e` + CR | Read encoders → `left right` + newline (see [WIRING.md](WIRING.md)) |
| `f` | `f` + CR | Read encoders **and** IMU in one round trip → `left right gx gy gz ax ay az imu_ok` |
| `g` | `g` + CR | Read IMU only → `gx gy gz ax ay az imu_ok` |
| `i` | `i` + CR | Re-estimate gyro bias (stops motors, blocks ~0.5 s) → `OK` or `IMU FAIL` |
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

Response on **`o` only:** `OK <L> <R>` (echoes received values). **`m` sends no reply.**

### `m` sends no reply

`m` arrives on every control frame and the Pi does not wait for an acknowledgement. Echoing `OK` per command filled the USB buffer and stalled the following `e` read, which showed up on the Pi as multi-hundred-millisecond `Read time` warnings, `Encoder read failed`, and Nav2 `Failed to make progress`. Do not add the reply back.

Commands that **do** reply: `c`, `w`, `x`, `r`, `u`, `i` → `OK`; `o` → `OK <L> <R>`; `b`, `e`, `f`, `g`, `a`, `d`, `p` → data; unknown → `Invalid Command`.

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

**Ki is the ramp rate.** At `Ko = 50`, a 1 tick/frame shortfall adds `Ki/Ko` PWM per frame, so **`Ki = 150` climbs about 150 PWM per second**. Tune via `u` (or `pid_i` in the Pi's `ros2_control.xacro`) without reflashing. Acceleration shaping belongs to `diff_drive_controller` on the Pi, not here.

### Default PID gains (buddy)

| Gain | Firmware default | Buddy `ros2_control.xacro` | Role |
|------|------------------|----------------------------|------|
| Kp | 100 | `pid_p` | Tracking — proportional to tick error |
| Kd | 40 | `pid_d` | Damping — reduces overshoot after breakaway |
| Ki | **150** | `pid_i` | **Ramp / breakaway** — pushes harder while stalled (raised for turns under body weight) |
| Ko | 50 | `pid_o` | Output scaling divisor |

Send: `u 100:40:150:50` + CR → `OK`.

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

### Firmware implementation (`imu_driver.ino`)

| Function / define | Location | Role |
|-------------------|----------|------|
| `IMU_I2C_SDA` / `SCL` | `imu_driver.h` | GPIO **21** / **22** |
| `imuInit()` | `imu_driver.ino` | Probe 0x68/0x69, configure gyro ±250 °/s, accel, bias average |
| `imuUpdate()` | `imu_driver.ino` | Called from main loop at **100 Hz** |
| `imuReadSerial()` | `imu_driver.ino` | Fills milli-unit values for `f` / `g` replies |
| `imuRecalibrateGyro()` | `imu_driver.ino` | Handler for serial **`i`** |
| `READ_STATE` / `READ_IMU` | `commands.h` | Command IDs for **`f`** and **`g`** |

Magnetometer (AK8963) is **never initialized** — indoor compass is unusable next to L298/motors.

### Run on Pi (after bench `g` → … `1`)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch buddy robot_navigation.launch.py map:=/path/to/map.yaml use_imu:=true
# ~20 s later, second terminal:
ros2 run buddy keyboard_teleop   # k then i — forward on map must match robot facing
```

### Bench scripts

```bash
~/esp/esp2ros2/scripts/test_closed_loop.sh /dev/ttyACM0 3   # closed-loop m (recommended)
~/esp/esp2ros2/scripts/test_motors.sh /dev/ttyACM0 130 130 2  # open-loop o
~/esp/esp2ros2/scripts/test_motors_diag.sh /dev/ttyACM0
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
| Encoder read | `e` every Nth frame (`encoder_read_divisor: 2` → 10 Hz) → `/diff_drive_controller/odom` → `odom_relay` → `/odom` |
| Encoder + IMU (`enable_imu: true`) | `f` at same 10 Hz → wheel odom + `/imu_sensor_broadcaster/imu`; EKF → fused `/odom` when `use_imu:=true` |
| Activate | `u P:D:I:Ko`, `i` if `imu_calibrate_on_activate`, then `m 0 0` on deactivate |

Key parameters in `buddy/description/ros2_control.xacro`:

| Parameter | Default | Notes |
|-----------|---------|-------|
| `use_open_loop_pwm` | false | Closed-loop `m` |
| `loop_rate` | 20 | Hz — matches `controller.yaml` |
| `encoder_read_divisor` | 2 | Poll `e` every 2nd frame |
| `motor_keepalive_ms` | 500 | Resend unchanged `m` (ESP auto-stop 2 s) |
| `pid_p/d/i/o` | 100/40/150/50 | Sent on activate |
| `enable_imu` | false | `true` → poll `f` instead of `e`; set from `use_imu:=true` at launch |
| `imu_calibrate_on_activate` | true | Send `i` on activate (no effect unless `enable_imu`) |
| `open_loop_min_pwm` / `max` | 130 / 230 | Open-loop path only |
| `swap_motor_pwm` | false | Do not double-cross with firmware `MOTOR_CROSS` |

Closed-loop driver: plain rad/s → ticks conversion — **no** tick slew, spin boost, or command filter.

Buddy motor tuning: [buddy/docs/DRIVE_TRAIN.md](https://github.com/mirzasalem/buddy-ros2/blob/main/docs/DRIVE_TRAIN.md) §3.

Only **one** process on the ESP serial port (close Serial Monitor / `screen` before `ros2 launch`).
