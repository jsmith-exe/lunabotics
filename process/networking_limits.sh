#!/bin/bash
# The following is only useful for the server or the robot itself.
# wondershaper is required; install it with these instructions:
# https://github.com/magnific0/wondershaper?tab=readme-ov-file#system-installation-optional

export DEFAULT_LIMITED_INTERFACE="tailscale0"

qpl_net_limit_set() {
  local EGRESS_PERCENT=${1:-90} # Default 90%
  local MAX_KBPS=${2:-4000} # Default 4000 Kbps (4 Mbps)
  local INTERFACE=${3:-DEFAULT_LIMITED_INTERFACE}

  local UPLOAD DOWNLOAD
  UPLOAD=$(echo "$MAX_KBPS * $EGRESS_PERCENT / 100" | bc)
  DOWNLOAD=$(echo "$MAX_KBPS - $UPLOAD" | bc)

  cat << EOF
Limiting $DEFAULT_LIMITED_INTERFACE to ${MAX_KBPS} Kbps
    Egress/upload: $UPLOAD Kbps (${EGRESS_PERCENT}%)
    Ingress/download: $DOWNLOAD Kbps ($((100 - EGRESS_PERCENT))%)
EOF

  sudo wondershaper -a "$INTERFACE" -d "$DOWNLOAD" -u "$UPLOAD"
}

qpl_net_limit_clear() {
  local INTERFACE=${3:-DEFAULT_LIMITED_INTERFACE}
  sudo wondershaper -c -a "$INTERFACE"
}

qpl_net_limit_status() {
  local INTERFACE=${3:-DEFAULT_LIMITED_INTERFACE}
  sudo wondershaper -s -a "$INTERFACE"
}
