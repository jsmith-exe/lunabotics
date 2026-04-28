# SERVER SPECIFIC INSTALLATION INSTRUCTIONS:
# The following is only useful for the server or the robot itself.

# wondershaper is used for network limiting; install it with these instructions:
# https://github.com/magnific0/wondershaper?tab=readme-ov-file#system-installation-optional

# Speedometer is used for network monitoring; install it with these instructions:
# https://excess.org/speedometer/
# Another good option is nload: sudo apt install nload

if [[ -z "$DEFAULT_LIMITED_INTERFACE" ]]; then
  export DEFAULT_LIMITED_INTERFACE="tailscale0"
fi

# --------------------- Network monitoring and limiting --------------------
alias qpl_speedometer="speedometer -s -l -m 625000 -r tailscale0 -t tailscale0"

qpl_net_limit_clear() {
  local INTERFACE=${1:-"$DEFAULT_LIMITED_INTERFACE"}

  for TABLE in iptables ip6tables; do
    # Remove jumps from main chains
    sudo $TABLE -D OUTPUT -o "$INTERFACE" -j QPL_LIMIT_OUT 2>/dev/null || true
    sudo $TABLE -D INPUT  -i "$INTERFACE" -j QPL_LIMIT_IN  2>/dev/null || true
    # Flush and delete custom chains
    sudo $TABLE -F QPL_LIMIT_OUT 2>/dev/null || true
    sudo $TABLE -F QPL_LIMIT_IN  2>/dev/null || true
    sudo $TABLE -X QPL_LIMIT_OUT 2>/dev/null || true
    sudo $TABLE -X QPL_LIMIT_IN  2>/dev/null || true
  done
}

qpl_net_limit_set() {
  local EGRESS_PERCENT=${1:-90}
  local MAX_KBPS=${2:-4000}
  local INTERFACE=${3:-"$DEFAULT_LIMITED_INTERFACE"}

  local UPLOAD DOWNLOAD
  UPLOAD=$(echo "$MAX_KBPS * $EGRESS_PERCENT / 100" | bc)
  DOWNLOAD=$(echo "$MAX_KBPS - $UPLOAD" | bc)

  # hashlimit uses kilobytes/sec — divide kbps by 8
  local UPLOAD_KBS=$(echo "$UPLOAD / 8" | bc)
  local DOWNLOAD_KBS=$(echo "$DOWNLOAD / 8" | bc)
  local UPLOAD_BURST_KBS=$UPLOAD_KBS
  local DOWNLOAD_BURST_KBS=$DOWNLOAD_KBS

  cat << EOF
Limiting $INTERFACE to ${MAX_KBPS} Kbps
    Egress/upload:    $UPLOAD Kbps (${EGRESS_PERCENT}%)
    Ingress/download: $DOWNLOAD Kbps ($((100 - EGRESS_PERCENT))%)
EOF

  qpl_net_limit_clear "$INTERFACE"

  for TABLE in iptables ip6tables; do
    # Create dedicated chains so we don't touch other rules
    sudo $TABLE -N QPL_LIMIT_OUT
    sudo $TABLE -N QPL_LIMIT_IN
    # Jump into them from the main chains
    sudo $TABLE -A OUTPUT -o "$INTERFACE" -j QPL_LIMIT_OUT
    sudo $TABLE -A INPUT  -i "$INTERFACE" -j QPL_LIMIT_IN
    # Egress limit
    sudo $TABLE -A QPL_LIMIT_OUT \
      -m hashlimit \
      --hashlimit-name "qpl_egress" \
      --hashlimit-above "${UPLOAD_KBS}kb/s" \
      --hashlimit-burst "${UPLOAD_BURST_KBS}kb" \
      -j DROP
    # Ingress limit
    sudo $TABLE -A QPL_LIMIT_IN \
      -m hashlimit \
      --hashlimit-name "qpl_ingress" \
      --hashlimit-above "${DOWNLOAD_KBS}kb/s" \
      --hashlimit-burst "${DOWNLOAD_BURST_KBS}kb" \
      -j DROP
  done
}

qpl_net_limit_status() {
  for TABLE in iptables ip6tables; do
    echo "=== $TABLE ==="
    echo "--- Jump rules (shows interface) ---"
    sudo $TABLE -L OUTPUT -v -n | grep QPL_LIMIT_OUT
    sudo $TABLE -L INPUT  -v -n | grep QPL_LIMIT_IN
    echo "--- Egress rules ---"
    sudo $TABLE -L QPL_LIMIT_OUT -v -n 2>/dev/null || echo "  (no chain)"
    echo "--- Ingress rules ---"
    sudo $TABLE -L QPL_LIMIT_IN  -v -n 2>/dev/null || echo "  (no chain)"
    echo
  done
}


# -------------------- DDS and other config --------------------
export ROS_DOMAIN_ID=42

use_server_flag_file_path="${QPL_PROJECT}/.use_server_sim"

use_server_sim() {
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file://"${QPL_PROJECT}"/dds/cyclonedds.xml
  touch "$use_server_flag_file_path" # Create file flag
  echo "Configured to use server sim. Type 'use_local_sim' to use local simulation."
}

use_local_sim() {
  unset RMW_IMPLEMENTATION
  unset CYCLONEDDS_URI
  rm -f "$use_server_flag_file_path"
  echo "Configured to use local sim. Type 'use_server_sim' to use server simulation."
}

if [ -f "$use_server_flag_file_path" ]; then
  use_server_sim
else
  use_local_sim
fi