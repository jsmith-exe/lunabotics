#!/bin/bash
# Builds a Jetson kernel with the traffic control modules wondershaper needs (ifb, sch_htb, sch_sfq,
# cls_u32, ...), plus NVIDIA's out-of-tree drivers and the WCH ch341 serial driver for that kernel.
# Run on the Jetson itself; nothing is installed. See docs/jetson-network-limiting.md.
#
# Usage: build_ifb_kernel.sh <source dir> [kernel|oot|ch341|all]
#   <source dir> must contain kernel_src.tbz2, kernel_oot_modules_src.tbz2 and
#   nvidia_kernel_display_driver_source.tbz2, extracted from NVIDIA's public_sources.tbz2.
set -euo pipefail

SRC_DIR=$(realpath "${1:?Usage: $0 <source dir> [kernel|oot|ch341|all]}")
STAGE=${2:-all}
BUILD_DIR=${BUILD_DIR:-$HOME/qpl_kernel}
CH341_SRC=${CH341_SRC:-$HOME/ch341ser_linux/driver}
KERNEL_TREE="$BUILD_DIR/kernel/kernel-jammy-src"
# Appended to the kernel version (5.15.148-tegra-ifb) so the new kernel and its modules install
# alongside the stock ones instead of replacing them
LOCALVERSION="-tegra-ifb"

# Modules wondershaper needs. Enabling IFB also enables CONFIG_NET_REDIRECT, which changes struct sk_buff;
# that's why a full kernel rebuild is needed rather than building these modules on their own.
MODULE_OPTIONS=(IFB NET_SCH_HTB NET_SCH_SFQ NET_CLS_U32 NET_SCH_TBF NET_SCH_FQ_CODEL NET_ACT_POLICE)

build_kernel() {
  mkdir -p "$BUILD_DIR"
  tar xjf "$SRC_DIR/kernel_src.tbz2" -C "$BUILD_DIR"
  cd "$KERNEL_TREE"

  # Start from the running kernel's config so everything else stays the same
  zcat /proc/config.gz > .config
  cp .config ../config.orig
  for OPTION in "${MODULE_OPTIONS[@]}"; do
    scripts/config --module "$OPTION"
  done
  scripts/config --set-str LOCALVERSION "$LOCALVERSION"
  make olddefconfig

  echo "=== Config changes from the running kernel:"
  scripts/diffconfig ../config.orig .config

  make -j"$(nproc)" Image modules
}

build_oot() {
  cd "$BUILD_DIR"
  tar xjf "$SRC_DIR/kernel_oot_modules_src.tbz2"
  tar xjf "$SRC_DIR/nvidia_kernel_display_driver_source.tbz2"
  # Builds hwpm, nvidia-oot, nvgpu and the display driver against the new kernel tree
  make KERNEL_HEADERS="$KERNEL_TREE" modules
}

build_ch341() {
  # The rover uses WCH's ch341 USB serial driver instead of the kernel's own one
  rm -rf "$BUILD_DIR/ch341"
  mkdir -p "$BUILD_DIR/ch341"
  cp "$CH341_SRC"/ch341.c "$CH341_SRC"/ch341.h "$CH341_SRC"/Makefile "$BUILD_DIR/ch341/"
  make -C "$BUILD_DIR/ch341" KERNELDIR="$KERNEL_TREE"
}

case "$STAGE" in
  kernel) build_kernel ;;
  oot) build_oot ;;
  ch341) build_ch341 ;;
  all) build_kernel; build_oot; build_ch341 ;;
  *) echo "Unknown stage: $STAGE"; exit 1 ;;
esac
