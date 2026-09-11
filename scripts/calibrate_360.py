#!/usr/bin/env python3
"""Gyro-referenced 360-degree calibration for buddy.

Answers two questions without a tape measure or a marked wheel:

  1. How many encoder ticks does one full WHEEL revolution take?
     (that is `encoder_counts_per_rev` in buddy/description/drive_train.xacro)
  2. How many encoder ticks does a full 360-degree in-place ROBOT spin take?

Ground truth comes from the MPU9250 on the ESP32, not from the encoders being
calibrated, so this is a genuinely independent check. The "f" command returns
encoders and gyro in one round trip, and the firmware AVERAGES gyro samples
over the poll interval (imuReadAveraged), so summing gz*dt between polls is an
exact integral of the mean rate -- the host poll rate does not bias the result.

Measured 2026-09-07 on the real robot, both directions, weight on the wheels:

    ticks per wheel revolution              358    (config: 358)
    ticks per wheel, full robot 360 spin   1482    (config predicts 1481.8)
    same spin re-run at the 1.6 rad/s cap  1484    (0.13% -- no wheel slip)

The spin figure calibrates the PRODUCT (wheel_separation / wheel_diameter) *
counts_per_rev. Both directions agreeing, and matching the configured geometry,
means encoder + track-width calibration is correct -- so if the robot feels
slow, look at the velocity/acceleration caps in controller.yaml and
nav2_params.yaml, not here. See buddy docs/DRIVE_TRAIN.md section 6.

Usage: python3 calibrate_360.py [port]

Stop `ros2 launch` and any other serial user first -- only one process may hold
the port. The robot spins in place, so it needs room to rotate but will not
drive away. Requires USE_IMU in the firmware (imu_ok must read 1).
"""
import math
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
GAINS = "100:80:350:50"      # matches ros2_control.xacro pid_p/d/i/o
TARGET = 10                  # ticks/frame per wheel -- slow enough to sample well
PID_RATE = 20                # must match firmware PID_RATE and Pi loop_rate

# Current config, for the comparison printed at the end.
CFG_CPR = 358.0
CFG_WHEEL_DIA = 0.069
CFG_SEPARATION = 0.2856      # controller.yaml wheel_separation * multiplier

s = serial.Serial(PORT, 115200, timeout=0.05)
time.sleep(2.0)              # ESP32 resets when the port opens


def send(cmd, wait=0.15):
    s.reset_input_buffer()
    s.write((cmd + "\r").encode())
    time.sleep(wait)
    return s.read(s.in_waiting or 1).decode(errors="replace").replace("\r", "").strip()


def poll():
    """One 'f' round trip -> (t, encL, encR, gz_rad_s), or None on a short read."""
    s.reset_input_buffer()
    s.write(b"f\r")
    deadline = time.time() + 0.15
    buf = b""
    while time.time() < deadline:
        buf += s.read(64)
        if b"\n" in buf:
            break
    t = time.time()
    p = buf.decode(errors="replace").replace("\r", "").strip().split()
    if len(p) < 9:
        return None
    try:
        return t, int(p[0]), int(p[1]), int(p[4]) / 1000.0
    except ValueError:
        return None


def spin(sign, label):
    """Spin in place until the integrated gyro yaw passes 360 degrees."""
    print(f"\n=== {label}: m {sign * TARGET} {-sign * TARGET} ===")
    send(f"u {GAINS}")
    send("r")
    print("   gyro bias (hold still):", send("i", 1.8))
    send("r")

    yaw = 0.0
    last = None
    started = time.time()
    stopped = False
    stop_time = 0.0
    keepalive = 0.0
    # Cut power early: PWM 0 brakes the L298, but momentum still carries the robot
    # ~15-25 deg further at this spin rate.
    lead_deg = 14.0

    while True:
        now = time.time()
        if not stopped and now - keepalive > 0.4:   # ESP auto-stops after 2 s
            s.write(f"m {sign * TARGET} {-sign * TARGET}\r".encode())
            keepalive = now
        r = poll()
        if r is None:
            continue
        t, _, _, gz = r
        if last is not None:
            yaw += gz * (t - last)                  # mean rate * interval
        last = t
        if not stopped and abs(math.degrees(yaw)) >= 360.0 - lead_deg:
            s.write(b"m 0 0\r")
            stopped = True
            stop_time = time.time()
        if stopped and time.time() - stop_time > 1.5:
            break
        if time.time() - started > 30:
            print("   TIMEOUT -- is the robot free to rotate?")
            s.write(b"m 0 0\r")
            break

    time.sleep(0.3)
    r = poll()
    if r is None:
        print("   could not read final encoders")
        return None
    _, fl, fr, _ = r
    deg = abs(math.degrees(yaw))
    if deg < 10:
        print(f"   gyro saw only {deg:.1f} deg -- check imu_ok and that the wheels turned")
        return None
    scale = 360.0 / deg
    l360, r360 = abs(fl) * scale, abs(fr) * scale
    print(f"   gyro yaw   : {deg:.1f} deg")
    print(f"   raw ticks  : L={fl:+d}  R={fr:+d}")
    print(f"   per 360    : L={l360:.0f}  R={r360:.0f}  mean={(l360 + r360) / 2:.0f}")
    return (l360 + r360) / 2


print(f"port {PORT}  target m {TARGET}  PID_RATE {PID_RATE}")
probe = poll()
if probe is None or send("f").split()[-1] != "1":
    print("\nWARNING: IMU not reporting ok (imu_ok != 1). This script needs USE_IMU.")

print("\nSpin in place -- give the robot room to rotate, then hands off.")
results = [r for r in (spin(+1, "SPIN A (CCW)"), None) if r is not None]
time.sleep(1.5)
results += [r for r in (spin(-1, "SPIN B (CW)"),) if r is not None]

s.write(b"m 0 0\r")
time.sleep(0.2)
s.close()

if results:
    spin_ticks = sum(results) / len(results)
    # A full robot spin rolls each wheel along a circle of diameter == the track,
    # so wheel revolutions per robot turn = wheel_separation / wheel_diameter.
    wheel_revs = CFG_SEPARATION / CFG_WHEEL_DIA
    measured_cpr = spin_ticks / wheel_revs
    predicted = wheel_revs * CFG_CPR
    print("\n" + "=" * 62)
    print(f"ticks per full robot 360 spin (per wheel) : {spin_ticks:.0f}")
    print(f"config predicts                           : {predicted:.0f}"
          f"   ({100 * (spin_ticks - predicted) / predicted:+.2f}%)")
    print(f"implied encoder_counts_per_rev            : {measured_cpr:.0f}"
          f"   (drive_train.xacro has {CFG_CPR:.0f})")
    print(f"ticks per metre travelled                 : "
          f"{CFG_CPR / (math.pi * CFG_WHEEL_DIA):.1f}")
    print("=" * 62)
    if abs(spin_ticks - predicted) / predicted < 0.02:
        print("Within 2% -- encoder and track-width calibration are good.")
        print("If the robot feels slow, the caps in controller.yaml /")
        print("nav2_params.yaml are the thing to change, not these.")
    else:
        print("Off by more than 2% -- update encoder_counts_per_rev in")
        print("drive_train.xacro, or wheel_separation_multiplier in")
        print("controller.yaml. See docs/DRIVE_TRAIN.md section 6.")
