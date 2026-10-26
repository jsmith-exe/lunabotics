#!/bin/bash
# Read-only check of can0: confirms the settings from can_setup.sh are live, then watches the
# error counters for a while. It sends nothing on the bus.
#
# Usage: ./can_health_check.sh [seconds]   (default 30)
# Run it while the ROS stack is driving the motors; errors mostly show up when the Jetson transmits.

IFACE=can0
DURATION=${1:-30}
fail=0

details=$(ip -details -statistics link show "$IFACE" 2>/dev/null)
if [ -z "$details" ]; then
  echo "FAIL: $IFACE not found"
  exit 1
fi

check() { # name, expected, actual
  if [ "$2" == "$3" ]; then
    echo "  ok    $1 = $3"
  else
    echo "  FAIL  $1 = $3 (expected $2)"
    fail=1
  fi
}

field() { # value following a keyword
  grep -oE "$1 [^ ]+" <<<"$details" | head -1 | awk '{print $2}'
}

counters() { # restarts bus-errors arbit-lost error-warn error-pass bus-off, then tx packets
  ip -details -statistics link show "$IFACE" | awk '
    /re-started/ {getline; c=$1" "$2" "$3" "$4" "$5" "$6}
    /TX:/ {getline; tx=$2}
    END {print c, tx}'
}

echo "Settings:"
check "state"        "UP"      "$(grep -q 'state UP\|,UP' <<<"$details" && echo UP || echo DOWN)"
check "bitrate"      "1000000" "$(field bitrate)"
check "sample-point" "0.800"   "$(field sample-point)"
check "sjw"          "10"      "$(field sjw)"
check "restart-ms"   "1"       "$(field restart-ms)"
check "qlen"         "100"     "$(field qlen)"
echo "  info  controller state: $(grep -oE 'state (ERROR-[A-Z]+|BUS-OFF|STOPPED)' <<<"$details" | awk '{print $2}')"

echo
echo "Watching error counters for ${DURATION}s..."
read -r _ be0 _ ew0 ep0 bo0 tx0 <<<"$(counters)"
sleep "$DURATION"
read -r _ be1 _ ew1 ep1 bo1 tx1 <<<"$(counters)"

d_tx=$((tx1 - tx0)); d_be=$((be1 - be0)); d_ep=$((ep1 - ep0)); d_bo=$((bo1 - bo0))
echo "  frames sent by Jetson: $d_tx"
echo "  bus errors:            $d_be"
echo "  error-passive events:  $d_ep"
echo "  bus-offs:              $d_bo"
echo

if [ "$d_be" -ne 0 ] || [ "$d_ep" -ne 0 ] || [ "$d_bo" -ne 0 ]; then
  echo "RESULT: FAIL - the bus is still producing errors"
  exit 1
elif [ "$d_tx" -eq 0 ]; then
  echo "RESULT: INCONCLUSIVE - the Jetson sent nothing; start the ROS stack and run again"
  exit 2
elif [ "$fail" -ne 0 ]; then
  echo "RESULT: BUS CLEAN, but the settings above don't match can_setup.sh (reboot or re-run it)"
  exit 1
else
  echo "RESULT: PASS - no bus errors while the Jetson sent $d_tx frames"
fi
