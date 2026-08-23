#!/usr/bin/env bash
# Bench-test ESP32 closed-loop PID (m command) without ROS.
# Usage: ./test_closed_loop.sh [port] [step_seconds]
# Example: ./test_closed_loop.sh /dev/ttyUSB1 3
#
# Robot on floor (4 kg load) or lifted for spin-only checks.
# Pass: wheels start slowly and build up over ~1 s (PWM is no longer floored at 130);
#       a wheel held by hand is pushed progressively harder; clean stop on m 0 0.
# Flash firmware first: ~/esp/esp2ros2/firmware/ROSArduinoBridge/

set -euo pipefail

PORT="${1:-/dev/ttyUSB1}"
SEC="${2:-3}"

if [[ ! -e "$PORT" ]]; then
  echo "Error: $PORT not found. ESP is usually ttyUSB1 (lidar often ttyUSB0)." >&2
  exit 1
fi

if fuser "$PORT" >/dev/null 2>&1; then
  echo "Error: $PORT is in use. Stop ros2 launch / keyboard_teleop / Serial Monitor." >&2
  fuser -v "$PORT" 2>&1 || true
  exit 1
fi

# Strip CR/LF — ESP Serial.println ends lines with \r\n; leftover \r breaks $(( )).
strip_cr() {
  printf '%s' "$1" | tr -d '\r\n'
}

# Read one reply line, strip CR, drop empty.
read_line() {
  local line
  line=$(timeout 0.6 head -1 <&3 2>/dev/null || true)
  strip_cr "$line"
}

send_cmd() {
  printf '%s\r' "$1" >&3
  sleep 0.25
  read_line
}

# "m" is fire-and-forget: the firmware sends no reply, so do not wait for one.
send_motor() {
  printf '%s\r' "$1" >&3
  sleep 0.1
}

read_encoders() {
  printf 'e\r' >&3
  sleep 0.2
  read_line
}

# Return only a signed integer, or empty if the field is garbage.
as_int() {
  local v
  v=$(strip_cr "$1")
  if [[ "$v" =~ ^-?[0-9]+$ ]]; then
    printf '%s' "$v"
  else
    printf ''
  fi
}

encoder_delta() {
  local before="$1"
  local after="$2"
  local side="$3"
  local b a
  if [[ "$side" == "L" ]]; then
    b=$(as_int "$(echo "$before" | awk '{print $1}')")
    a=$(as_int "$(echo "$after" | awk '{print $1}')")
  else
    b=$(as_int "$(echo "$before" | awk '{print $2}')")
    a=$(as_int "$(echo "$after" | awk '{print $2}')")
  fi
  if [[ -z "$a" || -z "$b" ]]; then
    echo "?"
    return 0
  fi
  echo $((a - b))
}

echo "Port=$PORT  step=${SEC}s — use floor for turn accuracy, or lift for spin-only."
echo "Waiting 2s for ESP32 boot after opening serial..."

exec 3<>"$PORT"
stty -F "$PORT" 115200 raw -echo -ixon cs8 -cstopb -parenb 2>/dev/null || true
sleep 2

echo ""
echo "=== Baud check ==="
send_cmd "b"

echo ""
echo "=== PID gains (match ros2_control.xacro: P:D:I:Ko) ==="
echo "I term is the ramp — raise it to break away sooner, lower it for a gentler start."
send_cmd "u 100:40:100:50"

echo ""
echo "=== Reset encoders ==="
send_cmd "r"

run_m() {
  local name="$1"
  local l="$2"
  local r="$3"
  local dur="${4:-$SEC}"
  echo ""
  echo "=== $name: m $l $r (${dur}s) ==="
  send_motor "m $l $r"
  echo "Encoders before:"
  local enc_before
  enc_before=$(read_encoders)
  echo "$enc_before"
  sleep "$dur"
  echo "Encoders after:"
  local enc_after
  enc_after=$(read_encoders)
  echo "$enc_after"
  local dl dr
  dl=$(encoder_delta "$enc_before" "$enc_after" L)
  dr=$(encoder_delta "$enc_before" "$enc_after" R)
  echo "Delta L=$dl R=$dr (expect sign match m $l $r; no runaway after m 0 0)"
  send_motor "m 0 0"
  sleep 0.8
  echo "Encoders after stop:"
  read_encoders
}

run_m "Slow forward" 5 5
run_m "Pivot turn accuracy (m 4 -4)" 4 -4 2
run_m "Arc turn accuracy (m 6 10)" 6 10 2
run_m "Slow in-place spin" 8 -8
run_m "Very slow spin" 3 -3
run_m "Differential turn (left faster)" 8 4

exec 3>&-
echo ""
echo "Done."
echo "- m 4 -4 / m 6 10: check encoder deltas match command; robot should stop cleanly on m 0 0."
echo "- Wheels should ease in, not snap: PWM now climbs from zero via the I term."
echo "- Hold a wheel by hand: it should fight back harder the longer you hold it."
echo "- Never moves at all? Raise I (u 100:40:200:50). Overshoots/hunts? Lower I or raise D."
