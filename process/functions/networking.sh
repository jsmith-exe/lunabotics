# SERVER SPECIFIC INSTALLATION INSTRUCTIONS:
# The following is only useful for the server or the robot itself.

# Speedometer is used for network monitoring; install it with these instructions:
# https://excess.org/speedometer/
# Another good option is nload: sudo apt install nload

# wondershaper is used for network limiting (upload and download); install it with these instructions:
# https://github.com/magnific0/wondershaper?tab=readme-ov-file#system-installation-optional
# The stock Jetson kernel is missing the modules wondershaper needs (ifb, sch_htb, sch_sfq, cls_u32);
# see docs/jetson-network-limiting.md for how to build a kernel that has them.

# The rover is on wifi; eno1 is the unused wired port
export DEFAULT_LIMITED_INTERFACE="wlP1p1s0"

# --------------------- Network monitoring and limiting --------------------
_qpl_net_redirecting_interfaces() {
  # Interfaces with an ingress qdisc, i.e. ones wondershaper is redirecting to ifb0
  sudo tc qdisc show | awk '$2 == "ingress" { for (i = 1; i < NF; i++) if ($i == "dev") print $(i + 1) }'
}

qpl_speedometer() {
  local INTERFACE=${1:-"$DEFAULT_LIMITED_INTERFACE"}
  speedometer -s -l -m 625000 -r "$INTERFACE" -t "$INTERFACE"
}

qpl_net_limit_set() {
  local EGRESS_PERCENT=${1:-90}
  local MAX_KBITS=${2:-4000}
  local INTERFACE=${3:-"$DEFAULT_LIMITED_INTERFACE"}

  local UPLOAD_KBITS DOWNLOAD_KBITS
  UPLOAD_KBITS=$(echo "$MAX_KBITS * $EGRESS_PERCENT / 100" | bc)
  DOWNLOAD_KBITS=$(echo "$MAX_KBITS - $UPLOAD_KBITS" | bc)

  # Validate
  local MIN_UPLOAD_KBITS MIN_DOWNLOAD_KBITS
  MIN_UPLOAD_KBITS=100
  MIN_DOWNLOAD_KBITS=100
  # Report only the rate that's actually too low, and fail non-zero so callers can tell
  if (( UPLOAD_KBITS < MIN_UPLOAD_KBITS )); then
    echo "UPLOAD_KBITS cannot be less than $MIN_UPLOAD_KBITS ($UPLOAD_KBITS)" >&2
    return 1
  fi
  if (( DOWNLOAD_KBITS < MIN_DOWNLOAD_KBITS )); then
    echo "DOWNLOAD_KBITS cannot be less than $MIN_DOWNLOAD_KBITS ($DOWNLOAD_KBITS)" >&2
    return 1
  fi

  qpl_net_limit_clear "$INTERFACE" >/dev/null 2>/dev/null

  # wondershaper shapes every interface's ingress through one shared ifb0, so a second limited interface
  # would fail to install its own shaper and later clearing either one would tear down the other's
  # downloads. Only one interface can be limited at a time, so drop any other one first.
  local OTHER
  for OTHER in $(_qpl_net_redirecting_interfaces); do
    if [[ "$OTHER" != "$INTERFACE" ]]; then
      echo "Clearing the existing limit on $OTHER (only one interface can be limited at a time)"
      sudo wondershaper -c -a "$OTHER" >/dev/null 2>&1
    fi
  done

  # wondershaper shapes egress with a hierarchical token bucket (htb) qdisc on the interface.
  # Ingress can't be queued directly, so wondershaper redirects incoming traffic to an intermediate
  # functional block device (ifb0) and shapes that device's egress with htb instead. This delays packets
  # rather than dropping them on arrival, which TCP handles far more smoothly. Rates are in Kbps.
  # Note: only IPv4 ingress is redirected to ifb0, so IPv6 downloads are not limited.
  sudo wondershaper -a "$INTERFACE" -u "$UPLOAD_KBITS" -d "$DOWNLOAD_KBITS"

  # wondershaper ignores the exit status of every tc command it runs and always succeeds, so check that
  # the qdiscs actually landed. On a stock Jetson kernel they don't -- see docs/jetson-network-limiting.md.
  if [[ $(qpl_net_limit_status_simple "$INTERFACE") != "Limited" ]]; then
    echo "Failed to limit $INTERFACE: the shaping qdiscs were not created." >&2
    echo "Is the network limiting kernel booted? See docs/jetson-network-limiting.md." >&2
    return 1
  fi

  cat << EOF
Limiting $INTERFACE to ${MAX_KBITS} Kbps
    Egress/upload:    $UPLOAD_KBITS Kbps (${EGRESS_PERCENT}%)
    Ingress/download: $DOWNLOAD_KBITS Kbps ($((100 - EGRESS_PERCENT))%)
EOF
}

qpl_net_limit_clear() {
  local INTERFACE=${1:-"$DEFAULT_LIMITED_INTERFACE"}

  echo "Clearing limits on $INTERFACE"
  sudo wondershaper -c -a "$INTERFACE"

  # Remove the iptables ingress limiter used by earlier versions of these functions, if still present
  for TABLE in iptables ip6tables; do
    sudo $TABLE -D INPUT -i "$INTERFACE" -j QPL_LIMIT_IN 2>/dev/null
    sudo $TABLE -F QPL_LIMIT_IN 2>/dev/null
    sudo $TABLE -X QPL_LIMIT_IN 2>/dev/null
  done
}

qpl_net_limit_status() {
  local INTERFACE=${1:-"$DEFAULT_LIMITED_INTERFACE"}

  echo "=== Egress/upload (tc with $INTERFACE) ==="
  sudo wondershaper -s -a "$INTERFACE"

  echo ""
  echo "=== Ingress/download (tc with ifb0) ==="
  sudo tc -s qdisc show dev ifb0 2>/dev/null || echo "  (no ifb0 device)"
  sudo tc -s class show dev ifb0 2>/dev/null
}

qpl_net_limit_status_simple() {
  local INTERFACE=${1:-"$DEFAULT_LIMITED_INTERFACE"}

  local HAS_EGRESS HAS_INGRESS
  # Egress shaping lives on the interface itself
  HAS_EGRESS=$(sudo tc qdisc show dev "$INTERFACE" | grep -c htb)
  # Downloads are only limited if the interface still redirects to ifb0 *and* ifb0 is still shaping:
  # clearing another interface deletes ifb0's qdisc while leaving this one's redirect in place
  HAS_INGRESS=0
  if sudo tc qdisc show dev "$INTERFACE" | grep -q ingress &&
     sudo tc qdisc show dev ifb0 2>/dev/null | grep -q htb; then
    HAS_INGRESS=1
  fi

  if [[ $HAS_EGRESS -gt 0 || $HAS_INGRESS -gt 0 ]]; then
    echo "Limited"
  else
    echo "Unlimited"
  fi
}


# -------------------- DDS and other config --------------------
export ROS_DOMAIN_ID=42

_get_highest_eth_interface() {
  # Function for WSL; this returns the highest eth interface, which is typically the one connected to
  # the network (e.g. eth0 is often a virtual interface for WSL itself).
  ip -brief addr show | grep UP | grep -oP 'eth[1-9]\d*' | sort -t'h' -k2 -n | tail -1
}
export QPL_WSL_INTERFACE=$(_get_highest_eth_interface)

qpl_dds_selector() {
  python3 "${QPL_PROJECT}/dds/selector.py"
  qpl_load_dds
}

qpl_load_dds() {
  set -a # Enable exporting all set variables
  source "${QPL_PROJECT}/dds/.current_dds" > /dev/null 2>&1 # Source current DDS config if it exists, ignore if not
  set +a # Disable exporting variables
}
qpl_load_dds

qpl_echo_dds() {
  echo "CURRENT_DDS: $CURRENT_DDS"
  echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
  echo "CYCLONEDDS_URI: $CYCLONEDDS_URI"
}
