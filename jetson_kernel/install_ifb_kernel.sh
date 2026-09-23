#!/bin/bash
# Installs the kernel built by build_ifb_kernel.sh alongside the stock kernel and makes it the default boot
# entry. The stock kernel, its modules and /boot/initrd are left untouched, and stay in the boot menu.
# Run on the Jetson. See docs/jetson-network-limiting.md.
#
# Usage: sudo install_ifb_kernel.sh <build dir>     e.g. sudo ./install_ifb_kernel.sh ~/qpl_kernel
#        sudo install_ifb_kernel.sh --revert        boot the stock kernel by default again
set -euo pipefail

EXTLINUX=/boot/extlinux/extlinux.conf
# The running kernel is the stock one whose NVIDIA modules get reused. Hardcoding a release here goes stale
# on every JetPack update and silently selects no modules, so default to whatever is booted. Override when
# already booted into a previously installed IFB kernel.
STOCK_RELEASE=${STOCK_RELEASE:-$(uname -r)}
# Label of the stock entry in extlinux.conf; its kernel arguments are reused and --revert boots it
STOCK_LABEL=${STOCK_LABEL:-primary}

if [[ $EUID -ne 0 ]]; then
  echo "Run with sudo"
  exit 1
fi

if [[ "${1:-}" == "--revert" ]]; then
  sed -i "s/^DEFAULT .*/DEFAULT $STOCK_LABEL/" "$EXTLINUX"
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
  echo "The new kernel's version must differ from the stock one ($STOCK_RELEASE) so it doesn't overwrite it."
  echo "If you are booted into a previously installed IFB kernel, boot the stock kernel first or pass"
  echo "STOCK_RELEASE=<stock release> explicitly."
  exit 1
fi

# Without this the module glob below quietly matches nothing and the kernel would be installed with none of
# NVIDIA's drivers -- no ethernet, no GPU, no display on a headless rover
if [[ ! -d "$STOCK_MODULES/updates" ]]; then
  echo "No stock NVIDIA modules at $STOCK_MODULES/updates."
  echo "Set STOCK_RELEASE to the release whose modules should be reused. Installed: $(ls -m /lib/modules)"
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
if [[ ! -s "$BUILD_DIR/oot_selection.tsv" ]]; then
  echo "No out-of-tree modules were selected from $STOCK_MODULES/updates; refusing to install a kernel that"
  echo "would boot without NVIDIA's drivers."
  exit 1
fi
while IFS=$'\t' read -r REL FILE SOURCE; do
  install -D -m 644 "$FILE" "$MODULES/updates/$REL"
done < "$BUILD_DIR/oot_selection.tsv"
# awk, not grep: under 'set -e' a grep that matches nothing would abort the install half-finished
awk -F'\t' '$3 == "rebuilt" { print "  rebuilt: " $1 }' "$BUILD_DIR/oot_selection.tsv"

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

# Reuse the stock entry's kernel arguments. Stop at the next LABEL so that a stock entry without an APPEND
# can't silently borrow the following entry's arguments.
APPEND=$(awk -v label="LABEL $STOCK_LABEL" '
  $0 ~ "^" label "[ \t]*$" { found = 1; next }
  found && /^LABEL / { exit }
  found && /^[ \t]*APPEND / { sub(/^[ \t]*APPEND[ \t]*/, ""); print; exit }
' "$EXTLINUX")
# An entry with an empty APPEND has no root=, so it would panic instead of booting
if [[ -z "$APPEND" ]]; then
  echo "Could not read APPEND from the '$STOCK_LABEL' entry in $EXTLINUX."
  echo "Set STOCK_LABEL to the label of the stock boot entry."
  exit 1
fi

# Drop the entry a previous run added (its LABEL line plus the indented lines under it) so this build's
# kernel is installed rather than leaving the default pointing at an older Image-<release>
awk '
  /^LABEL ifb[ \t]*$/ { skip = 1; next }
  skip && /^[ \t]*$/  { next }
  skip && /^[ \t]/    { next }
  { skip = 0; print }
' "$EXTLINUX" > "$EXTLINUX.tmp"
# Write through the existing file so its owner and mode are kept
cat "$EXTLINUX.tmp" > "$EXTLINUX"
rm -f "$EXTLINUX.tmp"

cat >> "$EXTLINUX" << EOF

LABEL ifb
      MENU LABEL kernel with network limiting modules ($RELEASE)
      LINUX /boot/Image-$RELEASE
      INITRD /boot/initrd-$RELEASE
      APPEND $APPEND
EOF
# TIMEOUT is in tenths of a second; 10s gives time to pick the stock kernel from the menu if needed
sed -i -e 's/^DEFAULT .*/DEFAULT ifb/' -e 's/^TIMEOUT .*/TIMEOUT 100/' "$EXTLINUX"

echo "Done. Reboot to use $RELEASE; 'sudo $0 --revert' makes the stock kernel the default again."
