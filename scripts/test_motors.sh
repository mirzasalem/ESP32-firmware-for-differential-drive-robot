#!/usr/bin/env bash
# Bench-test ESP32 motors without ROS.
# Usage: ./test_motors.sh [port] [left_pwm] [right_pwm] [seconds]
# Example: ./test_motors.sh /dev/ttyUSB1 230 230 3
# Buddy ROS caps PWM 130–230; below 130 may not move on this chassis.
#
# Do NOT use bare "printf ... > /dev/ttyUSB1" — each open resets ESP32 USB
# and the command is often lost. This script keeps the port open.

set -euo pipefail

PORT="${1:-/dev/ttyUSB1}"
LEFT="${2:-130}"
RIGHT="${3:-130}"
SEC="${4:-3}"

if [[ ! -e "$PORT" ]]; then
  echo "Error: $PORT not found. ESP is usually ttyUSB1 (lidar often ttyUSB0)." >&2
  exit 1
fi

if fuser "$PORT" >/dev/null 2>&1; then
  echo "Error: $PORT is in use. Stop ros2 launch / keyboard_teleop / Serial Monitor." >&2
  fuser -v "$PORT" 2>&1 || true
  exit 1
fi

echo "Port=$PORT  PWM left=$LEFT right=$RIGHT  duration=${SEC}s"
echo "Waiting 2s for ESP32 boot after opening serial..."

exec 3<>"$PORT"
stty -F "$PORT" 115200 raw -echo -ixon cs8 -cstopb -parenb 2>/dev/null || true
sleep 2

printf 'b\r' >&3
sleep 0.3
if ! timeout 1 cat <&3 | grep -q 115200; then
  echo "Warning: no baud response on $PORT — wrong device or firmware not running?" >&2
fi

printf 'o %s %s\r' "$LEFT" "$RIGHT" >&3
sleep 0.2
timeout 1 cat <&3 | head -1 || true
echo "Motors should run for ${SEC}s..."
sleep "$SEC"
printf 'o 0 0\r' >&3
sleep 0.2
exec 3>&-

echo "Done."
