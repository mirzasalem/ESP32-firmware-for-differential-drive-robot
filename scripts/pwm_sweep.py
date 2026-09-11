#!/usr/bin/env python3
"""Bench-characterize open-loop PWM breakaway and closed-loop low-target tracking
on the buddy ESP32.

Usage: python3 pwm_sweep.py [port] [pid_gains]
  port       default /dev/ttyACM0
  pid_gains  "Kp:Kd:Ki:Ko" sent via "u" before the closed-loop sweep,
             default matches ros2_control.xacro's current pid_p/d/i/o

Stop ros2 launch / test_closed_loop.sh first -- only one process may hold the
serial port. Both sweeps below command left/right in OPPOSITE directions
(spin-in-place, like m 4 -4) rather than straight-line driving, specifically
so this is safe to run with the robot's full weight on the floor without it
walking across the room during a ~40s sweep -- give it a little clearance to
rotate in place regardless. Restores the ESP's default gains before exiting
so a stray bench run doesn't leave a different tuning active for the next
`ros2 launch`.

Written 2026-09-02 while diagnosing "PWM too low to move, then it moves much
more than expected" -- see docs/DRIVE_TRAIN.md (top-of-file 2026-09-02 note)
and esp2ros2/docs/SERIAL_PROTOCOL.md ("Velocity PID and the PWM ramp"). First
run was unloaded (wheels lifted); this version is meant for the on-floor
re-verification that was still pending.
"""
import serial
import time
import sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
GAINS = sys.argv[2] if len(sys.argv) > 2 else "100:80:350:50"
RESTORE_GAINS = "100:80:350:50"  # current ros2_control.xacro default

s = serial.Serial(PORT, 115200, timeout=0.05)
time.sleep(2)


def send(cmd, wait=0.15):
    s.reset_input_buffer()
    s.write((cmd + "\r").encode())
    time.sleep(wait)
    return s.read(s.in_waiting or 1).decode(errors="replace").replace("\r", "").strip()


def read_encoders():
    s.reset_input_buffer()
    s.write(b"e\r")
    time.sleep(0.15)
    line = s.read(s.in_waiting or 1).decode(errors="replace").replace("\r", "").strip()
    parts = line.split()
    try:
        return int(parts[0]), int(parts[1])
    except Exception:
        return None, None


print("Spin-in-place sweep -- give the robot room to rotate, then hands off.")
print("baud:", send("b"))
print(f"gains u {GAINS} ->", send(f"u {GAINS}"))
print("reset:", send("r"))

print("\n=== Open-loop PWM sweep (o L -R), spin-in-place, 1.0s per step ===")
print(f"{'PWM':>5} {'dL':>6} {'dR':>6} {'L_ticks/s':>10} {'R_ticks/s':>10}")
for pwm in [20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 150]:
    l0, r0 = read_encoders()
    send(f"o {pwm} {-pwm}", wait=0.05)
    time.sleep(1.0)
    l1, r1 = read_encoders()
    send("o 0 0")
    time.sleep(0.5)
    dl, dr = l1 - l0, r1 - r0
    print(f"{pwm:>5} {dl:>6} {dr:>6} {dl:>10} {dr:>10}")

time.sleep(1)
print("\n=== Closed-loop low-target sweep (m L -R), spin-in-place, 3.0s per step ===")
print(f"{'target':>6} {'expected_ticks':>14} {'dL':>6} {'dR':>6} {'L_pct':>7} {'R_pct':>7}")
PID_RATE = 20
for tgt in [1, 2, 3, 4, 5, 6, 8, 10]:
    send("r")
    l0, r0 = read_encoders()
    dur = 3.0
    send(f"m {tgt} {-tgt}", wait=0.02)
    time.sleep(dur)
    l1, r1 = read_encoders()
    send("m 0 0")
    time.sleep(0.8)
    dl, dr = l1 - l0, r1 - r0
    expected = tgt * PID_RATE * dur
    lp = 100.0 * dl / expected if expected else 0
    rp = 100.0 * (-dr) / expected if expected else 0
    print(f"{tgt:>6} {expected:>14.0f} {dl:>6} {dr:>6} {lp:>6.1f}% {rp:>6.1f}%")

send("m 0 0")
send(f"u {RESTORE_GAINS}")
s.close()
print(f"\nRestored gains ({RESTORE_GAINS}). Done.")
