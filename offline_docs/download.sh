#!/bin/bash
# Downloads the offline documentation bank listed in index.md.
#
# Usage:
#   ./download.sh                 # every default section
#   ./download.sh dds can         # only some sections (optional ones included)
#   ./download.sh --list          # show the sections
#
# Re-running updates what's already there: git repos are re-fetched and web mirrors only
# re-download pages that changed. Failures don't stop the run; they're summarised at the end.
# Everything lands next to this script and is ignored by git (see .gitignore).
#
# Needs: git, wget, curl, tar. The default sections come to about 3.5 GB, most of it the Dash docsets
# and the Nav2 site (both image-heavy). The optional "web" section adds another ~2.3 GB.

set -u -o pipefail

DOCS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SECTIONS=(ros nav control localisation dds can jetson cameras network sim libs)
# Not downloaded by default; name them explicitly
OPTIONAL_SECTIONS=(web)
# Sparse-checkout patterns (gitignore syntax) for "the files at the repo root, but no directories"
ROOT_FILES=('/*' '!/*/')
FAILED=()

# -------------------- Helpers --------------------

log() { echo -e "\033[36m==> $*\033[0m"; }
fail() {
  echo -e "\033[31m  FAILED: $*\033[0m" >&2
  FAILED+=("$*")
}

# git_doc <dest> <url> <branch/tag> [sparse patterns...]
# Shallow clone of one branch or tag. With sparse patterns (gitignore syntax, e.g. "${ROOT_FILES[@]}"
# /doc/), only matching files are checked out and downloaded, which keeps big repos small.
git_doc() {
  local name=$1 dest="$DOCS_DIR/$1" url=$2 ref=$3
  shift 3
  log "git  $name ($ref)"
  if [[ -d "$dest/.git" ]]; then
    # Re-apply the patterns too, so changing them here takes effect on an existing clone
    git -C "$dest" fetch --quiet --depth 1 origin "$ref" &&
      git -C "$dest" reset --quiet --hard FETCH_HEAD &&
      { (( $# == 0 )) || git -C "$dest" sparse-checkout set --no-cone "$@"; } || fail "git update $dest"
    return
  fi
  mkdir -p "$(dirname "$dest")"
  if (( $# > 0 )); then
    git clone --quiet --depth 1 --filter=blob:none --no-checkout --branch "$ref" "$url" "$dest" &&
      git -C "$dest" sparse-checkout set --no-cone "$@" &&
      git -C "$dest" checkout --quiet || fail "git clone $url"
  else
    git clone --quiet --depth 1 --branch "$ref" "$url" "$dest" || fail "git clone $url"
  fi
}

# fetch <dest file> <url>: a single file, re-downloaded only if the server copy is newer
fetch() {
  local dest="$DOCS_DIR/$1" url=$2
  log "file $1"
  mkdir -p "$(dirname "$dest")"
  curl -fsSL --retry 3 -z "$dest" -o "$dest" "$url" || { fail "fetch $url"; return 1; }
}

# page <dest dir> <url>: one web page plus the images and CSS it needs, with links made local.
# Requisites may come from subdomains (e.g. files.waveshare.com) but not from other sites.
page() {
  local dest="$DOCS_DIR/$1" url=$2 host domain
  log "page $1"
  host=${url#*://}
  host=${host%%/*}
  domain=$(awk -F. '{print $(NF-1)"."$NF}' <<< "$host")
  # robots.txt is for crawlers; this fetches a single page, so ignore it (Waveshare's blocks wget)
  wget --quiet --timestamping --page-requisites --convert-links --adjust-extension --span-hosts --domains="$domain" \
    -e robots=off --no-directories -P "$dest" "$url"
  # As with mirror, a missing image shouldn't count as failure; the page itself missing should
  local file=${url##*/}
  [[ -f "$dest/${file%.html}.html" ]] || fail "page $url"
}

# mirror <dest dir> <url> [extra wget args...]: a whole site section below <url>
mirror() {
  local name=$1 dest="$DOCS_DIR/$1" url=$2
  shift 2
  log "site $name  ($url)"
  # wget exits non-zero if any single page 404s, which is normal on big sites, so only treat
  # "nothing downloaded at all" as a failure
  wget --quiet --recursive --level=inf --no-parent --timestamping --page-requisites \
    --convert-links --adjust-extension --wait=0.1 --tries=3 -P "$dest" "$@" "$url"
  [[ -n "$(find "$dest" -name '*.html' -print -quit 2>/dev/null)" ]] || fail "mirror $url"
}

# docset <name>: a Dash/Zeal docset. Open with Zeal, or browse
# <name>.docset/Contents/Resources/Documents/ directly in a browser.
docset() {
  local name=$1 dest="$DOCS_DIR/libs/docsets"
  log "docset $name"
  mkdir -p "$dest"
  curl -fsSL --retry 3 "https://kapeli.com/feeds/$name.tgz" | tar -xz -C "$dest" || fail "docset $name"
}

# -------------------- Sections --------------------

section_ros() {
  mirror ros/ros2-humble-html https://docs.ros.org/en/humble/ --reject-regex '/en/humble/p/'
  git_doc ros/ros2_documentation https://github.com/ros2/ros2_documentation.git humble "${ROOT_FILES[@]}" /source/
  mirror ros/rclpy-api https://docs.ros.org/en/humble/p/rclpy/
  git_doc ros/rclpy https://github.com/ros2/rclpy.git humble "${ROOT_FILES[@]}" /rclpy/docs/
  git_doc ros/rclcpp https://github.com/ros2/rclcpp.git humble "${ROOT_FILES[@]}" /rclcpp/doc/ /rclcpp/include/
  git_doc ros/image_common https://github.com/ros-perception/image_common.git humble
  git_doc ros/ffmpeg_image_transport https://github.com/ros-misc-utilities/ffmpeg_image_transport.git 3.0.4
  git_doc ros/xacro https://github.com/ros/xacro.git ros2
  git_doc ros/robot_state_publisher https://github.com/ros/robot_state_publisher.git humble
  git_doc ros/twist_mux https://github.com/ros-teleop/twist_mux.git humble
}

section_nav() {
  # gh-pages is the built site for jazzy and newer; Humble's isn't hosted any more, so take the
  # nearest (jazzy) and check parameter names against the Humble source below
  git_doc nav/nav2-docs-html https://github.com/ros-navigation/docs.nav2.org.git gh-pages /jazzy/
  git_doc nav/navigation2 https://github.com/ros-navigation/navigation2.git humble
  git_doc nav/spatio_temporal_voxel_layer https://github.com/SteveMacenski/spatio_temporal_voxel_layer.git humble
}

section_control() {
  # gh-pages holds every release's built site; only the Humble one is checked out
  git_doc control/control.ros.org-html https://github.com/ros-controls/control.ros.org.git gh-pages /humble/
  git_doc control/ros2_control https://github.com/ros-controls/ros2_control.git humble
  git_doc control/ros2_controllers https://github.com/ros-controls/ros2_controllers.git humble
}

section_localisation() {
  git_doc localisation/robot_localization https://github.com/cra-ros-pkg/robot_localization.git humble-devel
  # The wiki is mostly sample data and screenshots (~700 MB); the pages themselves are the .md files
  git_doc localisation/rtabmap.wiki https://github.com/introlab/rtabmap.wiki.git master '/*.md'
  git_doc localisation/rtabmap_ros https://github.com/introlab/rtabmap_ros.git humble-devel
  git_doc localisation/imu_tools https://github.com/CCNYRoboticsLab/imu_tools.git humble
  git_doc localisation/apriltag https://github.com/AprilRobotics/apriltag.git master
  git_doc localisation/pupil-apriltags https://github.com/pupil-labs/apriltags.git main
}

section_dds() {
  # 0.10.5 is the version ros-humble-cyclonedds ships
  mirror dds/cyclonedds-0.10.5-html https://cyclonedds.io/docs/cyclonedds/0.10.5/
  git_doc dds/cyclonedds https://github.com/eclipse-cyclonedds/cyclonedds.git 0.10.5 "${ROOT_FILES[@]}" /docs/
  git_doc dds/rmw_cyclonedds https://github.com/ros2/rmw_cyclonedds.git humble
}

section_can() {
  # REV publishes its whole docs site as one Markdown file, plus each page as .md
  fetch can/rev/rev-docs-full.md https://docs.revrobotics.com/llms-full.txt
  fetch can/rev/llms-index.md https://docs.revrobotics.com/llms.txt
  local url
  for url in $(grep -oE 'https://docs\.revrobotics\.com/(brushless/spark-max|brushless/legacy|revlib)[^)]*\.md' \
    "$DOCS_DIR/can/rev/llms-index.md" 2>/dev/null | sort -u); do
    fetch "can/rev/pages/${url#https://docs.revrobotics.com/}" "$url"
  done
  page can/frc-can-addressing https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html

  page can/waveshare-usb-can-a https://www.waveshare.com/wiki/USB-CAN-A
  local ws=https://files.waveshare.com/wiki/USB-CAN-A
  fetch "can/waveshare-usb-can-a/USB (Serial port) to CAN protocol defines.pdf" \
    "$ws/Demo/USB%20(Serial%20port)%20to%20CAN%20protocol%20defines.pdf"
  fetch can/waveshare-usb-can-a/USB-CAN-A-demo.zip "$ws/Demo/USB-CAN-A.zip"
  fetch can/waveshare-usb-can-a/USB-CAN-A-py.zip "$ws/Demo/python/USB-CAN-A-py.zip"
  fetch can/waveshare-usb-can-a/USBCANV2.12_English-windows-tool.zip "$ws/Tool/USBCANV2.12_English.zip"
  fetch can/waveshare-usb-can-a/CH341SER.zip https://files.waveshare.com/wiki/common/CH341SER.zip

  fetch can/linux-socketcan-can.rst https://raw.githubusercontent.com/torvalds/linux/v5.15/Documentation/networking/can.rst
  git_doc can/can-utils https://github.com/linux-can/can-utils.git master
}

section_jetson() {
  # The Jetson Linux Developer Guide for L4T R36.5 (JetPack 6). Large: a few hundred MB.
  mirror jetson/jetson-linux-r36.5 https://docs.nvidia.com/jetson/archives/r36.5/DeveloperGuide/index.html
}

section_cameras() {
  git_doc cameras/librealsense https://github.com/realsenseai/librealsense.git v2.58.4 "${ROOT_FILES[@]}" /doc/ /scripts/ /config/
  git_doc cameras/realsense-ros https://github.com/realsenseai/realsense-ros.git 4.58.4
  git_doc cameras/OrbbecSDK_ROS2 https://github.com/orbbec/OrbbecSDK_ROS2.git main
  git_doc cameras/pydualsense https://github.com/flok/pydualsense.git master
}

section_network() {
  git_doc network/wondershaper https://github.com/magnific0/wondershaper.git master
  mirror network/lartc-howto https://lartc.org/howto/
}

section_sim() {
  git_doc sim/gazebo_tutorials https://github.com/osrf/gazebo_tutorials.git master
  # Gazebo 11 uses SDFormat 9; the spec is the XML element descriptions under sdf/<version>/
  git_doc sim/sdformat https://github.com/gazebosim/sdformat.git sdf9 "${ROOT_FILES[@]}" /sdf/
  git_doc sim/gazebo_ros_pkgs https://github.com/ros-simulation/gazebo_ros_pkgs.git ros2
  git_doc sim/gazebo_ros2_control https://github.com/ros-controls/gazebo_ros2_control.git humble
}

section_libs() {
  fetch libs/python-3.10-docs-html.tar.bz2 https://docs.python.org/3.10/archives/python-3.10.22-docs-html.tar.bz2 &&
    tar -xjf "$DOCS_DIR/libs/python-3.10-docs-html.tar.bz2" -C "$DOCS_DIR/libs"
  local name
  for name in C++ CMake Bash NumPy OpenCV; do
    docset "$name"
  done
  page libs/ffmpeg https://ffmpeg.org/ffmpeg-all.html
  page libs/ffmpeg https://ffmpeg.org/ffmpeg-codecs.html
  git_doc libs/urwid https://github.com/urwid/urwid.git master "${ROOT_FILES[@]}" /docs/
  git_doc libs/ttkbootstrap https://github.com/israel-dryer/ttkbootstrap.git master "${ROOT_FILES[@]}" /docs/
}

# Optional: the HUD (basestation/hud) is plain JS/HTML/CSS. These docsets are ~2.3 GB together.
section_web() {
  local name
  for name in JavaScript HTML CSS; do
    docset "$name"
  done
}

# -------------------- Main --------------------

if [[ "${1:-}" == "--list" || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Sections: ${SECTIONS[*]}"
  echo "Optional (not in the default run): ${OPTIONAL_SECTIONS[*]}"
  echo "Usage: $0 [section...]   (no arguments downloads everything)"
  exit 0
fi

for tool in git wget curl tar; do
  command -v "$tool" >/dev/null || { echo "Missing required tool: $tool" >&2; exit 1; }
done

selected=("$@")
(( ${#selected[@]} == 0 )) && selected=("${SECTIONS[@]}")

for section in "${selected[@]}"; do
  if ! declare -F "section_$section" >/dev/null; then
    echo "Unknown section '$section'. Sections: ${SECTIONS[*]} ${OPTIONAL_SECTIONS[*]}" >&2
    exit 1
  fi
  echo -e "\n\033[1m######## $section ########\033[0m"
  "section_$section"
done

echo
du -sh "$DOCS_DIR" 2>/dev/null
if (( ${#FAILED[@]} > 0 )); then
  echo -e "\033[31m${#FAILED[@]} item(s) failed:\033[0m"
  printf '  %s\n' "${FAILED[@]}"
  exit 1
fi
echo "All done."
