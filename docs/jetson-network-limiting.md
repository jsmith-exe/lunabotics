# Network limiting on the Jetson

The `qpl_net_*` functions in `process/functions/networking.sh` limit the rover's upload and download bandwidth
using [wondershaper](https://github.com/magnific0/wondershaper). The stock JetPack kernel doesn't include the
modules wondershaper needs, so the Jetson runs a rebuilt kernel that has them. This document explains why and
how to build and install it.

## Why a custom kernel

Upload (egress) is easy to shape: `tc` queues outgoing packets and sends them at the configured rate.
Download (ingress) can't be queued on arrival. The earlier approach dropped packets over the limit with
iptables `hashlimit`, which TCP reacts to badly, so download limiting was flaky.

wondershaper shapes upload with an HTB qdisc, and handles download by redirecting incoming packets to an
IFB (intermediate functional block) virtual device, where they're shaped with HTB like outgoing traffic.
This needs these kernel modules:

| Module | Kernel option | Stock kernel (L4T R36.5.0, `5.15.185-tegra`) |
|---|---|---|
| `sch_htb` | `CONFIG_NET_SCH_HTB` | missing |
| `sch_sfq` | `CONFIG_NET_SCH_SFQ` | missing |
| `cls_u32` | `CONFIG_NET_CLS_U32` | missing |
| `ifb` | `CONFIG_IFB` | missing |
| `sch_ingress` | `CONFIG_NET_SCH_INGRESS` | present |
| `act_mirred` | `CONFIG_NET_ACT_MIRRED` | present |

`sch_htb`, `sch_sfq` and `cls_u32` could be built on their own against the installed kernel headers (as was
done for `sch_tbf` in `~/sch_tbf`). `ifb` can't: it needs `CONFIG_NET_REDIRECT`, a built-in option that
changes `struct sk_buff`, so it can only be turned on by rebuilding the whole kernel. Because `struct sk_buff`
changes, most other modules have to be rebuilt for the new kernel too (see below).

To check what a Jetson has:

```bash
zcat /proc/config.gz | grep -E 'CONFIG_(NET_REDIRECT|IFB|NET_SCH_HTB|NET_SCH_SFQ|NET_CLS_U32|NET_SCH_INGRESS|NET_ACT_MIRRED)[= ]'
```

## What gets installed

The rebuilt kernel is the stock release with an `-ifb` suffix, so on the current `5.15.185-tegra` it is
`5.15.185-tegra-ifb`. It's installed **alongside** the stock kernel, which is left untouched. `<release>` below
is the stock release that `install_ifb_kernel.sh` reads from `uname -r`:

| | Stock kernel | Rebuilt kernel |
|---|---|---|
| Kernel image | `/boot/Image` | `/boot/Image-<release>-ifb` |
| Initrd | `/boot/initrd` | `/boot/initrd-<release>-ifb` (copy of the stock one, plus the new kernel's modules) |
| Modules | `/lib/modules/<release>` | `/lib/modules/<release>-ifb` |
| Boot menu entry | `primary` | `ifb` (the default) |

The kernel config is the running kernel's config (`/proc/config.gz`) plus `IFB`, `NET_SCH_HTB`, `NET_SCH_SFQ`,
`NET_CLS_U32`, `NET_SCH_TBF`, `NET_SCH_FQ_CODEL` and `NET_ACT_POLICE` as modules. `NET_REDIRECT` is enabled
automatically by `IFB`.

NVIDIA's out-of-tree drivers (GPU, display, ethernet, etc. in `updates/`) are handled by
`jetson_kernel/select_oot_modules.py`: stock drivers are reused unchanged unless a kernel symbol they use has
changed, in which case the copy rebuilt from source is installed instead. That keeps as much as possible
identical to what NVIDIA shipped. The WCH `ch341` USB serial driver (from `~/ch341ser_linux`) is rebuilt and
installed in place of the kernel's own `ch341`, as on the stock kernel.

In practice almost everything is rebuilt: `NET_REDIRECT` changes the CRC of 4169 of the kernel's 18031
exported symbols, not just networking ones, because `struct sk_buff` is part of the type definition of
anything that reaches `struct device`, `struct net_device` or `struct sock`. Only 6 of NVIDIA's 105 drivers
are reused. The 99 rebuilt drivers were checked against the stock ones: all have exactly the same exported
functions, device IDs and module parameters, and every function name in a stock driver is also in the rebuilt
one. The display driver version (540.4.0) matches the installed display libraries.

## Building

Everything is built on the Jetson itself with its own gcc 11.4 (NVIDIA's packages were built with a gcc 11.3
cross-compiler; see [Notes](#notes)), and the sources download straight onto it.

1. **Get NVIDIA's sources** for the installed L4T release. `cat /etc/nv_tegra_release` gives the release; the
   rover is on R36.5.0, so the source directory is `r36_Release_v5.0`.

   ```bash
   mkdir -p ~/jetson_sources && cd ~/jetson_sources
   curl -O https://developer.download.nvidia.com/embedded/L4T/r36_Release_v5.0/sources/public_sources.tbz2
   tar xjf public_sources.tbz2 Linux_for_Tegra/source/kernel_src.tbz2 \
     Linux_for_Tegra/source/kernel_oot_modules_src.tbz2 \
     Linux_for_Tegra/source/nvidia_kernel_display_driver_source.tbz2
   mv Linux_for_Tegra/source/*.tbz2 . && rm -rf Linux_for_Tegra
   ```

   The download is 221 MB and only the three inner tarballs are needed, so `public_sources.tbz2` can be
   deleted afterwards. If NVIDIA publishes no sources for the installed release -- there were none for
   R36.4.7 -- a neighbouring release's tarball is sometimes the same kernel, but verify it first with
   [Checking the sources match the installed release](#checking-the-sources-match-the-installed-release).

2. **Install ch341 for USB functionality**
   ```bash
   git clone https://github.com/WCHSoftGroup/ch341ser_linux ~/ch341ser_linux
   ```


3. **Build** (no sudo needed). On an Orin Nano the kernel takes about 30 minutes.

   ```bash
   ~/lunabotics/jetson_kernel/build_ifb_kernel.sh ~/jetson_sources all
   ```

   The build goes in `~/qpl_kernel` (override with `BUILD_DIR=...`). It prints the config changes from the
   running kernel; they should be the options above, `NET_REDIRECT`, `LOCALVERSION` and compiler-related
   entries only. The stages can also be run separately: `kernel`, `oot`, then `ch341`.

### Checking the sources match the installed release

Only needed when building from a tarball whose release doesn't match the installed one. JetPack installs the
kernel and driver sources on the Jetson at the same relative paths the tarballs extract to, so the two can be
compared directly:

| Installed on the Jetson | From package | Matching source tree |
|---|---|---|
| `/usr/src/kernel/kernel-jammy-src` | `nvidia-l4t-kernel-headers` | `kernel/kernel-jammy-src` (`kernel_src.tbz2`) |
| `/usr/src/nvidia/{hwpm,nvgpu,nvidia-oot}` | `nvidia-l4t-kernel-oot-headers` | `{hwpm,nvgpu,nvidia-oot}` (`kernel_oot_modules_src.tbz2`) |

The check walks the files the **Jetson** has, because neither side is a subset of the other: the installed
kernel tree is a built tree with ~1000 generated files no tarball has, while the installed NVIDIA trees are
only the 186 headers needed to build against, out of several thousand files in the tarball.

Extract the tarballs somewhere and point `SOURCES` at it:

```bash
SOURCES=~/jetson_sources/temp   # must contain kernel/kernel-jammy-src, hwpm, nvgpu and nvidia-oot
DIFFS=~/source-check-diffs.txt  # every differing file is recorded here
: > "$DIFFS"

check_tree() {
  local installed=$1 sources=$2 same=0 differ=0 extra=0 rel
  [[ -d "$sources" ]] || { echo "$(basename "$installed") -> no source tree at $sources"; return 1; }
  while IFS= read -r rel; do
    if [[ ! -e "$sources/$rel" ]]; then
      extra=$((extra + 1))
    elif cmp -s "$installed/$rel" "$sources/$rel"; then
      same=$((same + 1))
    else
      differ=$((differ + 1)); echo "$rel" >> "$DIFFS"
    fi
  done < <(find "$installed" -type f \( -name '*.c' -o -name '*.h' -o -name '*.S' \) \
             -not -name '*.mod.c' -not -path '*/generated/*' -not -name '.tmp_*' -printf '%P\n')
  echo "$(basename "$installed") -> $same identical, $differ differing, $extra not in sources"
}

check_tree /usr/src/kernel/kernel-jammy-src "$SOURCES/kernel/kernel-jammy-src"
for M in hwpm nvgpu nvidia-oot; do
  check_tree "/usr/src/nvidia/$M" "$SOURCES/$M"
done
```

It takes about 2.5 minutes. `: > "$DIFFS"` empties the log file first, so a run never shows leftovers from
the previous one.

#### Reading the output

Each tree reports three counts:

| Count | Meaning |
|---|---|
| `identical` | In both, byte-for-byte the same. |
| `differing` | In both, contents differ. **This is the verdict, and it must be 0.** |
| `not in sources` | The Jetson has the file, the tarball doesn't. Expected to be non-zero for the kernel tree. |

`not in sources` is not a failure. The installed kernel tree is a *built* tree, so it holds files that are
generated during a build but named like ordinary sources, and no pristine tarball will ever contain them. On
R36.5.0 there are 51: flex and bison output under `scripts/` (`parser.tab.c`, `lexer.lex.c`, the `dtc` and
`genksyms` parsers), ASN.1 compiler output (`crypto/*.asn1.c`, `fs/cifs/*.asn1.h`), `lib/raid6/{int,neon}*.c`
expanded from templates by `unroll.awk`, SELinux's `flask.h` and `av_permissions.h`, `lib/crc32table.h`,
`net/wireless/shipped-certs.c` and `arch/arm64/crypto/sha*-core.S`. The `*.mod.c`, `*/generated/*` and
`.tmp_*` files are skipped before this point; these 51 are what's left.

Only `differing` matters, because a changed source file changes symbol CRCs. That would make
`select_oot_modules.py` rebuild drivers it should have reused, leaving a result that doesn't match what NVIDIA
shipped.

**A pass** -- R36.5.0 sources against the installed R36.5.0, checked September 2026. Safe to build:

```
kernel-jammy-src -> 54259 identical, 0 differing, 51 not in sources
hwpm -> 1 identical, 0 differing, 0 not in sources
nvgpu -> 8 identical, 0 differing, 0 not in sources
nvidia-oot -> 177 identical, 0 differing, 0 not in sources
```

**A failure** -- the R36.4.4 tarball against the same Jetson. Do not build from these:

```
kernel-jammy-src -> 49446 identical, 4779 differing, 85 not in sources
hwpm -> 1 identical, 0 differing, 0 not in sources
nvgpu -> 8 identical, 0 differing, 0 not in sources
nvidia-oot -> 175 identical, 1 differing, 1 not in sources
```

R36.4.4 is kernel 5.15.148 and R36.5.0 is 5.15.185, so 4779 kernel files genuinely differ. Note how little the
NVIDIA trees give away -- `hwpm` and `nvgpu` look identical in both runs, because only 1 and 8 of their headers
are installed. The kernel tree is the meaningful signal. (The R36.4.4 substitution did work for R36.4.7, which
shipped 5.15.148 too; it only works when the kernel version is unchanged.)

## Installing

```bash
sudo ~/lunabotics/jetson_kernel/install_ifb_kernel.sh ~/qpl_kernel
sudo reboot
```

After the reboot, `uname -r` should print `<release>-ifb`, and
`sudo modprobe -a ifb sch_htb sch_sfq cls_u32` should succeed.

The installer takes the stock release from `uname -r` and the stock boot entry's kernel arguments from the
`primary` label in `extlinux.conf`. Override either if they don't apply, for example when re-running while
already booted into a previously installed IFB kernel:

```bash
sudo STOCK_RELEASE=5.15.185-tegra STOCK_LABEL=primary \
  ~/lunabotics/jetson_kernel/install_ifb_kernel.sh ~/qpl_kernel
```

The install script backs up `/boot/extlinux/extlinux.conf` before editing it, and sets the boot menu timeout to
10 seconds (from 3) so there's time to pick the stock kernel if needed.

### Going back to the stock kernel

- If the Jetson boots: `sudo ~/lunabotics/jetson_kernel/install_ifb_kernel.sh --revert`, then reboot.
- If it doesn't boot: connect an HDMI/DisplayPort monitor and a keyboard, and choose **primary kernel** in the
  boot menu within 10 seconds of power-on. Then run the revert command above.

The stock kernel and its modules are never modified, so either way the Jetson runs exactly as it did before. The
`ifb` entry stays in the boot menu (and the menu timeout stays at 10 seconds) until removed from
`/boot/extlinux/extlinux.conf`.

## Using network limiting

Install wondershaper once (it installs to `/usr/local/sbin`):

```bash
git clone https://github.com/magnific0/wondershaper.git
cd wondershaper && sudo make install
```

Then, from a shell with the `qpl_*` functions loaded:

```bash
qpl_net_limit_set 90 4000        # 4000 Kbps total on wlP1p1s0: 90% (3600 Kbps) up, 10% (400 Kbps) down
qpl_net_limit_set 50 4000 usb1   # a different interface
qpl_net_limit_status             # detailed tc statistics, including the ifb0 device used for download
qpl_net_limit_status_simple      # "Limited" or "Unlimited"
qpl_net_limit_clear
```

wondershaper only redirects IPv4 downloads to `ifb0`, so IPv6 downloads aren't limited.

## Test results

Tested on the Orin Nano (September 2026) over the USB network link to a PC (`usb0` on the Jetson), measuring
20-second SSH transfers in each direction while the limit was set:

| Command | Download target | Download measured | Upload target | Upload measured |
|---|---|---|---|---|
| `qpl_net_limit_set 50 4000 usb0` | 2000 Kbps | 1900 Kbps | 2000 Kbps | 1677 Kbps |
| `qpl_net_limit_set 90 4000 usb0` | 400 Kbps | 380 Kbps | 3600 Kbps | 2569 Kbps |
| `qpl_net_limit_clear usb0` | unlimited | 161 Mbps | unlimited | 283 Mbps |

The measured rates are application data, so they sit below the targets because of TCP and SSH overhead (and, for
upload in the second test, TCP acknowledgements having to fit through the 400 Kbps download limit). Neither
direction exceeded its limit.

After booting the rebuilt kernel: all drivers loaded with no module errors, a CUDA test program ran correctly on
the GPU, the WCH `ch341` driver was in use, and the only failed service (`vncserver.service`) was the same as on
the stock kernel. The display driver logs `failed to set bandwidth` and `nvidia-modeset: ERROR ... DRAM` messages
when no monitor is connected; the stock kernel logs the same messages.

## On the rover's Jetson

**The rebuilt kernel is not currently installed on the rover.** As of the L4T R36.5.0 upgrade there is no
`~/qpl_kernel` build tree, no `/boot/Image-*-ifb`, and `extlinux.conf` still has `DEFAULT primary`, so the
Jetson boots the stock kernel and `qpl_net_limit_set` will fail on the missing modules until the build in
[Building](#building) is redone. wondershaper itself is installed in `/usr/local/sbin`.

The scripts are in this repo at `~/lunabotics/jetson_kernel`, and the build goes in `~/qpl_kernel`. Once the
kernel is installed, going back to the stock one is
`sudo ~/lunabotics/jetson_kernel/install_ifb_kernel.sh --revert` plus a reboot.

## Notes

- `apt upgrade` of the `nvidia-l4t-kernel*` packages only updates the stock kernel. After a JetPack upgrade,
  rebuild with the matching sources, or switch back to the stock kernel.
- NVIDIA builds its kernel with a gcc 11.3 cross-compiler; this build uses the Jetson's own gcc 11.4, which drops
  `CONFIG_GCC_PLUGINS` (no plugins were enabled in the stock kernel, so nothing else changes).
- The rebuilt modules aren't signed with NVIDIA's key. Module signatures aren't enforced, so they load normally
  (the kernel reports itself as tainted with out-of-tree and unsigned modules, as it does with the stock setup).
- The stock kernel lacks `sch_fq_codel`, so Ubuntu's default queueing discipline setting
  (`net.core.default_qdisc = fq_codel`) silently fell back to `pfifo_fast`. The rebuilt kernel has it, so network
  interfaces now use `fq_codel` by default when not limited. This generally reduces latency under load.
