#!/bin/bash
: 'Cycles can0 when mttcan stops transmitting after a bus-off (TX counter frozen while frames wait in the
queue, every send failing with ENOBUFS). The plugin keeps its socket across down/up and resends every cycle,
so motors resume on their own; down also discards the stuck frames so no stale commands go out.
Settings (including the enable flag) are in can_watchdog.conf next to this script.
Runs as root from can0-watchdog.service (see that file for install steps).'

IFACE=${1:-can0}
CONFIG=${CONFIG:-$(dirname "$(readlink -f "$0")")/can_watchdog.conf}
POLL_MS=100                     # must match the sleep at the bottom of the loop

IP=$(command -v ip)
TC=$(command -v tc)
prev_tx=-1
frozen=0
restarts=0
prev_enabled=""

# Re-read every poll so the flag can be flipped without restarting the service.
load_config() {
  ENABLED=true; STALL_MS=300; COOLDOWN_S=1   # defaults if the file or a line is missing
  [ -r "$CONFIG" ] && source "$CONFIG"
  stall_polls=$(( STALL_MS / POLL_MS ))
  if [ "$ENABLED" != "$prev_enabled" ]; then
    if [ "$ENABLED" = true ]; then
      echo "can_watchdog: enabled, watching $IFACE (stall after ${STALL_MS} ms, config $CONFIG)"
    else
      echo "can_watchdog: disabled in $CONFIG; stalls are logged but $IFACE is not restarted"
    fi
    prev_enabled=$ENABLED
  fi
}

while true; do
  load_config

  tx=$(cat /sys/class/net/$IFACE/statistics/tx_packets 2>/dev/null)
  if [ -z "$tx" ]; then
    prev_tx=-1; frozen=0; sleep 1; continue   # interface missing (still being set up)
  fi

  # Only ask tc for the backlog when TX has not moved; an idle interface with nothing queued is not a stall.
  if [ "$tx" = "$prev_tx" ]; then
    backlog=$($TC -s qdisc show dev $IFACE | awk '/backlog/ {print $3+0; exit}')
    if [ "${backlog:-0}" -gt 0 ]; then frozen=$((frozen + 1)); else frozen=0; fi
  else
    frozen=0
  fi
  prev_tx=$tx

  if [ $frozen -ge $stall_polls ]; then
    state=$($IP -details link show $IFACE | grep -oE 'state [A-Z-]+ \(berr-counter[^)]*\)')
    if [ "$ENABLED" = true ]; then
      restarts=$((restarts + 1))
      echo "can_watchdog: $IFACE TX stalled (tx=$tx backlog=${backlog}p $state), cycling interface (restart $restarts)"
      $IP link set $IFACE down && $IP link set $IFACE up
      prev_tx=-1
    else
      echo "can_watchdog: $IFACE TX stalled (tx=$tx backlog=${backlog}p $state), not restarting (disabled)"
    fi
    frozen=0
    sleep $COOLDOWN_S
    continue
  fi

  sleep 0.1
done
