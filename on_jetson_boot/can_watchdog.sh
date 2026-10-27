#!/bin/bash
: 'Cycles can0 when mttcan stops transmitting after a bus-off (TX counter frozen while frames wait in the
queue, every send failing with ENOBUFS). The plugin keeps its socket across down/up and resends every cycle,
so motors resume on their own; down also discards the stuck frames so no stale commands go out.
Runs as root from can0-watchdog.service (see that file for install steps).'

IFACE=${1:-can0}
STALL_MS=${STALL_MS:-300}       # TX frozen with a backlog for this long counts as a stall
POLL_MS=100                     # must match the sleep at the bottom of the loop
COOLDOWN_S=${COOLDOWN_S:-1}     # wait after a restart, so a bus with no other nodes is not cycled every 300 ms

IP=$(command -v ip)
TC=$(command -v tc)
stall_polls=$(( STALL_MS / POLL_MS ))
prev_tx=-1
frozen=0
restarts=0

echo "can_watchdog: watching $IFACE (stall after ${STALL_MS} ms)"
while true; do
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
    restarts=$((restarts + 1))
    state=$($IP -details link show $IFACE | grep -oE 'state [A-Z-]+ \(berr-counter[^)]*\)')
    echo "can_watchdog: $IFACE TX stalled (tx=$tx backlog=${backlog}p $state), cycling interface (restart $restarts)"
    $IP link set $IFACE down && $IP link set $IFACE up
    frozen=0; prev_tx=-1
    sleep $COOLDOWN_S
    continue
  fi

  sleep 0.1
done
