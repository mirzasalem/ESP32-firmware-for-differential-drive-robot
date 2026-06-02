# Wiring — ESP32 + L298N + encoders

Default pin map: `firmware/ROSArduinoBridge/motor_driver.h`, `encoder_driver.h`.

## Power

```
Motor battery (7–12 V typical) ──► L298N +12V / VMS
                              └──► Motor outputs M1, M2

ESP32 USB ──► Raspberry Pi or PC (logic + serial only)

**Common GND** required: ESP32 GND ↔ L298N GND ↔ motor battery GND
```

- Do **not** power motors from the ESP32 3.3 V / 5 V pin.
- Pi and ESP32 share ground through USB; motor battery ground must tie in.

## L298N → ESP32 (motor GPIO)

| L298N label | ESP32 GPIO | Firmware name |
|-------------|------------|---------------|
| IN1 | **18** | LEFT direction A |
| IN2 | **19** | LEFT direction B |
| IN3 | **32** | RIGHT direction A |
| IN4 | **33** | RIGHT direction B |
| ENA | **25** or jumper 5 V | LEFT enable |
| ENB | **14** or jumper 5 V | RIGHT enable |
| GND | GND | Common ground |

Many boards ship with **ENA/ENB jumpers** (always on). Then only IN1–IN4 need wires.

### Physical motor outputs vs firmware LEFT/RIGHT

| L298N outputs | Buddy chassis |
|---------------|---------------|
| OUT1, OUT2 | **Physical left** wheel (+Y in URDF) |
| OUT3, OUT4 | **Physical right** wheel (−Y in URDF) |

Encoders are mounted on the same side as each wheel (LEFT GPIO 26/27 on left, RIGHT 16/17 on right).

### Buddy default: split motor / encoder cross (`buddy_robot_config.h`)

On this chassis the **L298 motor leads** are crossed relative to encoder sides. Buddy uses **two flags**:

| Flag | Default | Effect |
|------|---------|--------|
| `BUDDY_L298_ENCODER_CROSS` | **0** | Encoders direct → TF / `/odom` / AMCL correct |
| `BUDDY_L298_MOTOR_CROSS` | **1** | `setMotorSpeeds()` crosses PWM to L298 |

| ROS / serial | Encoder on `e` (ENCODER_CROSS=0) | Motor PWM (MOTOR_CROSS=1) |
|--------------|----------------------------------|---------------------------|
| Left joint (`o` arg1) | GPIO **26, 27** on **physical left** | OUT3/4 (firmware RIGHT) |
| Right joint (`o` arg2) | GPIO **16, 17** on **physical right** | OUT1/2 (firmware LEFT) |

**Do not** cross `left_wheel_names` in buddy `controller.yaml` or set `swap_motor_pwm: true` on the Pi when firmware uses this split — that double-swaps turns.

If **one wheel** runs backward for forward teleop only, swap **that motor’s two wires** or invert `left_motor_scale` / `right_motor_scale` in buddy `ros2_control.xacro`.

### Bench test (motors)

Use the scripts (keep USB open — do not `printf > /dev/ttyUSB1` alone):

```bash
~/esp/esp2ros2/scripts/test_motors_diag.sh /dev/ttyUSB1
~/esp/esp2ros2/scripts/test_motors.sh /dev/ttyUSB1 130 130 2
~/esp/esp2ros2/scripts/test_motors.sh /dev/ttyUSB1 230 230 2
```

Serial `o <L> <R>` always uses ROS joint order (left arg = `left_wheel_joint`).

| Command | Meaning (with buddy defaults) |
|---------|------------------------------|
| `o 130 130` | Both wheels forward (min buddy PWM) |
| `o 230 230` | Both wheels forward (max buddy PWM) |
| `o 130 0` | Left joint only → OUT3/4 (**physical right** motor) |
| `o 0 130` | Right joint only → OUT1/2 (**physical left** motor) |
| `o 130 -130` | Spin in place |

Keyboard **`j`** / **`l`** and Nav2 use standard diff-drive turns (no pivot hack on buddy).

Response: `OK <L> <R>` echoes the values received.

If **`o 130 130` works** but one side is weak, try PWM **230** (buddy `open_loop_max_pwm` on the Pi).

### Diag: last spin step fails (`o -130 130`)

After `test_motors_diag.sh`, compare these (with **MOTOR_CROSS=1**, **ENCODER_CROSS=0**):

| Step | Serial | Physical RIGHT (OUT3/4) | Physical LEFT (OUT1/2) |
|------|--------|-------------------------|-------------------------|
| ROS right only | `o 0 130` | off | forward |
| ROS right reverse | `o 0 -130` | off | reverse |
| ROS left only | `o 130 0` | forward | off |
| **ROS left reverse** | **`o -130 0`** | **reverse** | off |
| Spin other way | `o -130 130` | reverse | forward |

### OUT3/4 reverse (`o -130 0`) does not move or matches `o 130 0`

OUT1/2 and OUT3/4 use **identical** `drive_hbridge()` code in `motor_driver.ino` — no per-channel flags.

| Command | GPIO toggled | OUT1/2 (works) | OUT3/4 (broken) |
|---------|--------------|----------------|-----------------|
| `o 130 0` / `o 0 130` forward | LEFT_FWD 18 / RIGHT_FWD 32 | forward | forward |
| `o -130 0` / `o 0 -130` reverse | LEFT_BACK 19 / RIGHT_BACK 33 | reverse | **no motion / same direction** |

This is a **hardware** problem on the L298 channel B (IN4 / OUT3 / OUT4 / GPIO 33), not firmware. Check:

1. **Is GPIO 33 actually wired to L298 IN4?** Probe with a multimeter on `o -130 0` (should swing between ~0 V and ~3.3 V).
2. **L298 IN4 dead** — try a different L298 module; some cheap boards have one channel weak/dead.
3. **OUT3/OUT4 motor wires loose** — re-seat both screws on OUT3/4.
4. **If only one polarity drives the motor at all**, the L298 channel B is bad — replace the module.

Once `o -130 0` produces motion **opposite** to `o 130 0`, the firmware needs no changes. If the wheel direction is then **physically inverted** vs forward teleop (`i` goes backward on that side), set in `ros2_control.xacro`:

```xml
<param name="left_motor_scale">-1.0</param>
<param name="left_motor_scale_reverse">-1.0</param>
```

## Encoders → ESP32 (buddy default)

Configured in `encoder_driver.h` / `encoder_driver.ino`:

| Firmware label | GPIO A | GPIO B | Physical side | Notes |
|----------------|--------|--------|---------------|--------|
| **LEFT** | **26** | **27** | Left wheel | `BUDDY_LEFT_ENCODER_INVERT 0` (raw; set 1 to negate) |
| **RIGHT** | **16** | **17** | Right wheel | Count used as-is |

- Use **3.3 V** logic encoders; firmware enables **internal pull-ups** (`ESP32Encoder`).
- If you change encoder pins, re-upload and recalibrate buddy `encoder_counts_per_rev` in `drive_train.xacro`.

## USB on robot

| Device | Typical port |
|--------|----------------|
| RPLIDAR | `/dev/ttyUSB0` |
| ESP32 | `/dev/ttyUSB1` |

Buddy: `buddy/scripts/setup_usb_serial.sh` (udev) and `ros2_control.xacro` → `device`.

## Changing pins or cross flags

1. Edit `buddy_robot_config.h`, `motor_driver.h`, `encoder_driver.h` as needed
2. **Upload** `ROSArduinoBridge.ino` from the sketch folder
3. On buddy: update `enc_counts_per_rev`, `device`, `motor_scale`, `open_loop_*` only if needed — keep `swap_motor_pwm: false` when using split cross in firmware

## ASCII overview

```
                    ┌─────────────┐
   Motor battery ──►│   L298N     ├──► OUT1/2  → physical LEFT wheel
                    │  IN1..IN4   ├──► OUT3/4  → physical RIGHT wheel
                    └──────┬──────┘
                           │ GPIO 18,19,32,33
                    ┌──────▼──────┐
   Enc L: 26,27 ──►│   ESP32     │◄── USB ──► Pi (buddy / diffdrive_arduino)
   Enc R: 16,17 ──►└─────────────┘
        ENCODER_CROSS=0: left joint reads left encoder
        MOTOR_CROSS=1:   left joint PWM → OUT3/4 (crossed)
```
