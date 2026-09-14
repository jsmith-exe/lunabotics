#!/bin/bash
# Installs the kernel built by build_ifb_kernel.sh alongside the stock kernel and makes it the default boot
# entry. The stock kernel, its modules and /boot/initrd are left untouched, and stay in the boot menu.
# Run on the Jetson. See docs/jetson-network-limiting.md.
#
# Usage: sudo install_ifb_kernel.sh <build dir>     e.g. sudo ./install_ifb_kernel.sh ~/qpl_kernel
#        sudo install_ifb_kernel.sh --revert        boot the stock kernel by default again
set -euo pipefail

EXTLINUX=/boot/extlinux/extlinux.conf
STOCK_RELEASE=${STOCK_RELEASE:-5.15.148-tegra}

if [[ $EUID -ne 0 ]]; then
  echo "Run with sudo"
  exit 1
fi

if [[ "${1:-}" == "--revert" ]]; then
  sed -i 's/^DEFAULT .*/DEFAULT primary/' "$EXTLINUX"
  echo "Stock kernel is the default again; reboot to use it"
  exit 0
fi

BUILD_DIR=$(realpath "${1:?Usage: sudo $0 <build dir> | --revert}")
SCRIPT_DIR=$(dirname "$(realpath "$0")")
KERNEL_TREE="$BUILD_DIR/kernel/kernel-jammy-src"
RELEASE=$(cat "$KERNEL_TREE/include/config/kernel.release")
MODULES="/lib/modules/$RELEASE"
STOCK_MODULES="/lib/modules/$STOCK_RELEASE"

if [[ "$RELEASE" == "$STOCK_RELEASE" ]]; then
  echo "The new kernel's version must differ from the stock one ($STOCK_RELEASE) so it doesn't overwrite it"
  exit 1
fi

echo "=== Installing in-tree modules to $MODULES"
make -C "$KERNEL_TREE" -s modules_install

echo "=== Choosing NVIDIA out-of-tree modules"
python3 "$SCRIPT_DIR/select_oot_modules.py" \
  --kernel-symvers "$KERNEL_TREE/Module.symvers" \
  --stock-updates "$STOCK_MODULES/updates" \
  --rebuilt-dir "$BUILD_DIR/hwpm" \
  --rebuilt-dir "$BUILD_DIR/nvidia-oot" \
  --rebuilt-dir "$BUILD_DIR/nvgpu" \
  --rebuilt-dir "$BUILD_DIR/nvdisplay/kernel-open" \
  > "$BUILD_DIR/oot_selection.tsv"
while IFS=$'\t' read -r REL FILE SOURCE; do
  install -D -m 644 "$FILE" "$MODULES/updates/$REL"
done < "$BUILD_DIR/oot_selection.tsv"
grep -P '\trebuilt$' "$BUILD_DIR/oot_selection.tsv" | cut -f1 | sed 's/^/  rebuilt: /'

echo "=== Installing WCH ch341 driver in place of the kernel's own"
install -D -m 644 "$BUILD_DIR/ch341/ch341.ko" "$MODULES/kernel/drivers/usb/serial/ch341.ko"

echo "=== Running depmod (any unresolved symbols are listed below)"
depmod -a -e -F "$KERNEL_TREE/System.map" "$RELEASE"

echo "=== Installing /boot/Image-$RELEASE"
install -m 644 "$KERNEL_TREE/arch/arm64/boot/Image" "/boot/Image-$RELEASE"

echo "=== Creating /boot/initrd-$RELEASE"
# Same as NVIDIA's nv-update-initrd, except it adds the new kernel's modules to a copy of the stock initrd
# (keeping the stock kernel's modules too) instead of rewriting /boot/initrd
INITRD_DIR=$(mktemp -d)
trap 'rm -rf "$INITRD_DIR"' EXIT
(cd "$INITRD_DIR" && gunzip -c /boot/initrd | cpio -i --quiet)
grep -E "^[^#]" /etc/nv-update-initrd/list.d/modules | sed "s|<KERNEL_VERSION>|$RELEASE|g" |
  while IFS=: read -r SRC DST; do
    mkdir -p "$INITRD_DIR/${DST%/*}"
    # shellcheck disable=SC2086 # SRC may be a glob
    cp -f $SRC "$INITRD_DIR/$DST"
  done
(cd "$INITRD_DIR" && find . | cpio -H newc -o --quiet | gzip -9 -n) > "/boot/initrd-$RELEASE"

echo "=== Adding boot entry to $EXTLINUX"
cp "$EXTLINUX" "$EXTLINUX.bak-$(date +%Y%m%d-%H%M%S)"
if ! grep -q "^LABEL ifb" "$EXTLINUX"; then
  # Reuse the stock entry's kernel arguments
  APPEND=$(awk '/^LABEL primary/ {found = 1} found && /APPEND/ {sub(/^[ \t]*APPEND /, ""); print; exit}' "$EXTLINUX")
  cat >> "$EXTLINUX" << EOF

LABEL ifb
      MENU LABEL kernel with network limiting modules ($RELEASE)
      LINUX /boot/Image-$RELEASE
      INITRD /boot/initrd-$RELEASE
      APPEND $APPEND
EOF
fi
# TIMEOUT is in tenths of a second; 10s gives time to pick the stock kernel from the menu if needed
sed -i -e 's/^DEFAULT .*/DEFAULT ifb/' -e 's/^TIMEOUT .*/TIMEOUT 100/' "$EXTLINUX"

echo "Done. Reboot to use $RELEASE; 'sudo $0 --revert' makes the stock kernel the default again."
