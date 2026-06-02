#!/usr/bin/env bash
# Per-motor bench test (one USB session — no ESP reset between steps).
# Usage: ./test_motors_diag.sh [/dev/ttyUSB1] [PWM]
#
# Buddy defaults: ENCODER_CROSS=0, MOTOR_CROSS=1 in buddy_robot_config.h

set -euo pipefail

PORT="${1:-/dev/ttyUSB1}"
PWM="${2:-130}"

if [[ ! -e "$PORT" ]]; then
  echo "Error: $PORT not found." >&2
  exit 1
fi

if fuser "$PORT" >/dev/null 2>&1; then
  echo "Error: $PORT in use. Stop ros2 launch / teleop first." >&2
  exit 1
fi

send_o() {
  printf 'o %s %s\r' "$1" "$2" >&3
  sleep 0.25
  timeout 0.5 cat <&3 | head -1 || true
}

echo "Port=$PORT  PWM=$PWM — keep robot lifted or blocked."
echo "Opening serial (2s boot wait)..."

exec 3<>"$PORT"
stty -F "$PORT" 115200 raw -echo -ixon cs8 -cstopb -parenb 2>/dev/null || true
sleep 2

printf 'b\r' >&3
sleep 0.3
timeout 1 cat <&3 | head -1 || true

run_step() {
  local name="$1"
  local l="$2"
  local r="$3"
  local sec="${4:-2}"
  echo ""
  echo "=== $name: o $l $r (${sec}s) ==="
  send_o "$l" "$r"
  sleep "$sec"
  send_o 0 0
  sleep 0.3
}

run_step "Both forward (ROS L+R)" "$PWM" "$PWM"
run_step "ROS left joint only (MOTOR_CROSS: physical RIGHT / OUT3-4)" "$PWM" 0
run_step "ROS left reverse (OUT3-4, must oppose o $PWM 0)" "-$PWM" 0
run_step "ROS right joint only (MOTOR_CROSS: physical LEFT / OUT1-2)" 0 "$PWM"
run_step "ROS right reverse only (physical LEFT / OUT1-2)" 0 "-$PWM"
run_step "Spin: ROS L +$PWM R -$PWM" "$PWM" "-$PWM"
run_step "Spin: ROS L -$PWM R +$PWM" "-$PWM" "$PWM"

exec 3>&-
echo ""
echo "Done. Encoders stay direct (ENCODER_CROSS=0). See docs/WIRING.md"
