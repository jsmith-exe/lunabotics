#!/usr/bin/env bash
# Swap which physical camera drives the big teleop pane.
#
# The two rviz2 windows started by launch/teleop.launch.py read fixed topics
# (/teleop/main_cam and /teleop/pip_cam). This flips the two topic_tools/mux nodes
# that feed them, so the main pane and the corner pane exchange cameras.
#
#   cam_swap.sh          toggle
#   cam_swap.sh front    front camera to the main pane
#   cam_swap.sh rear     rear camera to the main pane
#
# Transport-agnostic: the actual topic names are read back from the mux itself, so
# this works whether the stack was launched with transport:=ffmpeg or :=compressed.
set -euo pipefail

MAIN_MUX=${MAIN_MUX:-/teleop_main_cam_mux}
PIP_MUX=${PIP_MUX:-/teleop_pip_cam_mux}

die() { echo "cam_swap: $*" >&2; exit 1; }

# MuxList returns the mux's configured inputs; pick out the front and rear ones so
# we never have to hardcode a transport suffix here.
inputs=$(ros2 service call "${MAIN_MUX}/list" topic_tools_interfaces/srv/MuxList 2>/dev/null \
         | grep -o "'/[^']*'" | tr -d "'") \
  || die "could not reach ${MAIN_MUX}/list -- is teleop.launch.py running?"

front=$(echo "$inputs" | grep -m1 front) || die "no front camera among the mux inputs: $inputs"
rear=$(echo "$inputs" | grep -m1 rear) || die "no rear camera among the mux inputs: $inputs"

select_topic() {  # mux, topic -> echoes the topic that was selected before
  ros2 service call "$1/select" topic_tools_interfaces/srv/MuxSelect "{topic: $2}" \
    | grep -o "prev_topic='[^']*'" | cut -d"'" -f2
}

case "${1:-toggle}" in
  front) want=$front ;;
  rear)  want=$rear ;;
  toggle)
    # Stateless toggle: selecting front tells us what was selected before, so if we
    # were already on front we just fall through to rear.
    prev=$(select_topic "$MAIN_MUX" "$front")
    if [ "$prev" = "$front" ]; then want=$rear; else want=$front; fi
    ;;
  *) die "usage: cam_swap.sh [front|rear]" ;;
esac

if [ "$want" = "$front" ]; then other=$rear; else other=$front; fi

select_topic "$MAIN_MUX" "$want" >/dev/null
select_topic "$PIP_MUX" "$other" >/dev/null
echo "main pane: $want"
echo "corner:    $other"
