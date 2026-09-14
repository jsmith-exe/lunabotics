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

| Module | Kernel option | Stock kernel (L4T R36.4.7) |
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

The rebuilt kernel is `5.15.148-tegra-ifb`. It's installed **alongside** the stock `5.15.148-tegra` kernel,
which is left untouched:

| | Stock kernel | Rebuilt kernel |
|---|---|---|
| Kernel image | `/boot/Image` | `/boot/Image-5.15.148-tegra-ifb` |
| Initrd | `/boot/initrd` | `/boot/initrd-5.15.148-tegra-ifb` (copy of the stock one, plus the new kernel's modules) |
| Modules | `/lib/modules/5.15.148-tegra` | `/lib/modules/5.15.148-tegra-ifb` |
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
cross-compiler; see [Notes](#notes)). The Jetson has no internet connection, so download the sources on another
computer and copy them over.

1. **Get NVIDIA's sources.** The Jetson runs JetPack 6.2.x, L4T R36.4.7 (check with `cat /etc/nv_tegra_release`).
   No R36.4.7 source download could be found, so the R36.4.4 sources were used:
   <https://developer.download.nvidia.com/embedded/L4T/r36_Release_v4.4/sources/public_sources.tbz2>.
   They're the same kernel: all 5805 kernel header files and all 186 NVIDIA driver source and header files that
   the R36.4.7 packages install on the Jetson are byte-identical to the R36.4.4 sources. If a later JetPack is
   installed, use its sources.

   ```bash
   # On another computer
   tar xjf public_sources.tbz2 Linux_for_Tegra/source/kernel_src.tbz2 \
     Linux_for_Tegra/source/kernel_oot_modules_src.tbz2 \
     Linux_for_Tegra/source/nvidia_kernel_display_driver_source.tbz2
   scp Linux_for_Tegra/source/*.tbz2 <user>@<jetson>:~/jetson_sources/
   scp -r jetson_kernel <user>@<jetson>:~/
   ```

2. **Build** (on the Jetson; no sudo needed). On an Orin Nano the kernel takes about 30 minutes.

   ```bash
   ~/jetson_kernel/build_ifb_kernel.sh ~/jetson_sources all
   ```

   The build goes in `~/qpl_kernel` (override with `BUILD_DIR=...`). It prints the config changes from the
   running kernel; they should be the options above, `NET_REDIRECT`, `LOCALVERSION` and compiler-related
   entries only. The stages can also be run separately: `kernel`, `oot`, then `ch341`.

## Installing

```bash
sudo ~/jetson_kernel/install_ifb_kernel.sh ~/qpl_kernel
sudo reboot
```

After the reboot, `uname -r` should print `5.15.148-tegra-ifb`, and
`sudo modprobe -a ifb sch_htb sch_sfq cls_u32` should succeed.

The install script backs up `/boot/extlinux/extlinux.conf` before editing it, and sets the boot menu timeout to
10 seconds (from 3) so there's time to pick the stock kernel if needed.

### Going back to the stock kernel

- If the Jetson boots: `sudo ~/jetson_kernel/install_ifb_kernel.sh --revert`, then reboot.
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
qpl_net_limit_set 90 4000        # 4000 Kbps total on eno1: 90% (3600 Kbps) upload, 10% (400 Kbps) download
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

The build, logs and scripts are in `~/qpl_kernel` (scripts in `~/qpl_kernel/jetson_kernel`), so to go back to the
stock kernel run `sudo ~/qpl_kernel/jetson_kernel/install_ifb_kernel.sh --revert` and reboot. wondershaper is
installed in `/usr/local/sbin`. The rebuilt kernel is the default boot entry.

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
