# SERVER SPECIFIC INSTALLATION INSTRUCTIONS:
# The following is only useful for the server or the robot itself.

# Speedometer is used for network monitoring; install it with these instructions:
# https://excess.org/speedometer/
# Another good option is nload: sudo apt install nload

# DEPRECATED:
# wondershaper was used for network limiting; install it with these instructions:
# https://github.com/magnific0/wondershaper?tab=readme-ov-file#system-installation-optional

export DEFAULT_LIMITED_INTERFACE="eno1"

# --------------------- Network monitoring and limiting --------------------
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
  if (( UPLOAD_KBITS < MIN_UPLOAD_KBITS|| MIN_DOWNLOAD_KBITS < 100 )); then
    echo "UPLOAD_KBITS cannot be less than $MIN_UPLOAD_KBITS ($UPLOAD_KBITS)"
    echo "DOWNLOAD_KBITS cannot be less than $MIN_DOWNLOAD_KBITS ($DOWNLOAD_KBITS)"
    return
  fi

  # hashlimit uses kilobytes/sec — divide kbps by 8
  local DOWNLOAD_KBYTES=$(echo "$DOWNLOAD_KBITS / 8" | bc)

  qpl_net_limit_clear "$INTERFACE" >/dev/null 2>/dev/null

  cat << EOF
Limiting $INTERFACE to ${MAX_KBITS} Kbps
    Egress/upload:    $UPLOAD_KBITS Kbps (${EGRESS_PERCENT}%)
    Ingress/download: $DOWNLOAD_KBITS Kbps ($((100 - EGRESS_PERCENT))%)
EOF

  # Use tc for egress
  # Add queuing disciplice (qdisc) to interface/device (dev) to the egress/outbound (root),
  # using token bucket filter (tbf), with the rate at which the bucket fills (rate).
  # Key params:
  # rate     — how fast the bucket refills
  # burst    — how large the bucket is
  # latency  — how long a packet waits before being dropped if bucket is empty
  sudo tc qdisc add dev "$INTERFACE" root tbf \
    rate "${UPLOAD_KBITS}kbit" \
    burst 15360 \
    latency 50ms

  # Use iptables; limit IPv4 and 6
  for TABLE in iptables ip6tables; do
    # Creates a new 'chain' - think of this as a pipeline of sorts
    sudo $TABLE -N QPL_LIMIT_IN
    # Add (-A) rule to INPUT chain, for interface (-i), redirect (-j) to QPL_LIMIT_IN chain
    sudo $TABLE -A INPUT -i "$INTERFACE" -j QPL_LIMIT_IN
    # Add rule to our new QPL_LIMIT_IN chain for limiting network by dropping traffic
    sudo $TABLE -A QPL_LIMIT_IN \
      -m hashlimit \
      --hashlimit-name "qpl_ingress" \
      --hashlimit-above "${DOWNLOAD_KBYTES}kb/s" \
      --hashlimit-burst "${DOWNLOAD_KBYTES}kb" \
      -j DROP
  done
}

qpl_net_limit_clear() {
  local INTERFACE=${1:-"$DEFAULT_LIMITED_INTERFACE"}

  # Remove egress shaping
  echo "Clearing egress on $INTERFACE"
  sudo tc qdisc del dev "$INTERFACE" root

  # Remove ingress iptables chains
  for TABLE in iptables ip6tables; do
    echo "Clearing egress $TABLE on $INTERFACE"
    # Delete rule (D), flush (F), delete chain (X)
    sudo $TABLE -D INPUT -i "$INTERFACE" -j QPL_LIMIT_IN 
    sudo $TABLE -F QPL_LIMIT_IN 
    sudo $TABLE -X QPL_LIMIT_IN 
  done
}

qpl_net_limit_status() {
  local INTERFACE=${1:-"$DEFAULT_LIMITED_INTERFACE"}

  echo "=== Egress (tc with $INTERFACE) ==="
  sudo tc -s qdisc show dev "$INTERFACE"

  echo ""
  for TABLE in iptables ip6tables; do
    echo "=== Ingress ($TABLE) ==="
    echo "--- Jump rule ---"
    sudo $TABLE -L INPUT -v -n | grep QPL_LIMIT_IN
    echo "--- Ingress rules ---"
    sudo $TABLE -L QPL_LIMIT_IN -v -n 2>/dev/null || echo "  (no chain)"
    echo ""
  done
}

qpl_net_limit_status_simple() {
  local INTERFACE=${1:-"$DEFAULT_LIMITED_INTERFACE"}

  local HAS_EGRESS HAS_INGRESS
  # Count for expected values if enabled
  HAS_EGRESS=$(sudo tc qdisc show dev "$INTERFACE" | grep -c tbf)
  HAS_INGRESS=$(sudo iptables -L QPL_LIMIT_IN -v -n 2>/dev/null | grep -c hashlimit)

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
  ip -brief addr show | grep -oP 'eth[1-9]\d*' | sort -t'h' -k2 -n | tail -1
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
  echo "CURRENT_DDS: ${CURRENT_DDS:-Unset}"
}
qpl_load_dds

qpl_echo_dds() {
  echo "CURRENT_DDS: $CURRENT_DDS"
  echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
  echo "CYCLONEDDS_URI: $CYCLONEDDS_URI"
}
