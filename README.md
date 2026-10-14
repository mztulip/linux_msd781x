Experimental Linux fork for running it on MSD7816/MSD7818 (kernel 6.7.0-rc6)

[.config](https://gist.github.com/maniekx86/abbbdef38c84eae7efed8118df665ca3) that I used to compile and run this

Use mipsel toolchain to compile this. Also I recommend to check arch/mips/boot/dts/mstar/msd7816-test.dts too

Default bootargs: `console=ttyS0,115200 root=/dev/sda2 rootwait clk_ignore_unused rw fbcon=font:TER16x32`

## Changes in this fork (WIWA HD95, MSD7818)

Tested on a WIWA HD95 DVB-T box (MSD7818, MxL5007T tuner). Ready-made
`vmlinux.bin`, modules and boot instructions:
https://github.com/mztulip/msd7818_rootfs_kernel

- **I2C driver** `drivers/i2c/busses/i2c-mstar-miic.c` (`CONFIG_I2C_MSTAR_MIIC`,
  compatible `mstar,msd7816-miic`). The Kronus hardware I2C master is a
  Philips SIO1 style engine, so `i2c-msc313e` does not fit. Pinmux via the
  chiptop syscon: `mstar,chiptop`, `mstar,port`, `mstar,pad-mode` (port 0,
  mode 1 = tuner bus, MxL5007T at 0x60). Module parameter `pinmux_16bit`.
- **Front panel driver** `drivers/input/keyboard/mstar-fd650-panel.c`
  (`CONFIG_KEYBOARD_MSTAR_FD650`): FD650/TM1650 style controller on PM SAR
  pins 1 (DIO) and 2 (CLK). Keys POWER / CH+ / CH- as an input device,
  4-digit display via sysfs `text` and `brightness`.
- **Build both drivers as modules** (`=m`). Built in (`=y`) they make the
  HDMI picture jitter; loaded with `insmod` after boot they do not.
- **DTS** `msd7816-test.dts`: `&miic0` pinmux properties, front panel node,
  removed the g781 / ds1340 nodes (nothing answers there).
- **Framebuffer colours**: `drm_fb_blit()` no longer forces an RGB565 ->
  xRGB4444 conversion. MBoot's `osd_create 1 1` sets GOP format 2, which is
  ARGB4444 (format 1 is RGB565, 9 is YUV422). The boot helper `logofix`
  (see the rootfs repo) converts the boot logo and leaves GOP3 in real RGB565,
  so the kernel copies RGB565 as is and gets 65536 colours.
  **This kernel needs `logofix` instead of `osd_create`** in `bootlinux`,
  otherwise colours are wrong.
- **Double buffered framebuffer** `drivers/video/fbdev/mstar-gopfb.c`
  (`CONFIG_FB_MSTAR_GOP`, built in, replaces simpledrm): `/dev/fb0` is
  1920x1080 RGB565 with `yres_virtual` = 2160. `FBIOPAN_DISPLAY` switches
  buffers at vsync, `FBIO_WAITFORVSYNC` waits for it (the output runs at
  50 Hz). The DTS reserves 9 MB at `0x2100000` (`fb-memory`).
- **Config**: `arch/mips/configs/wiwa_hd95_defconfig`.

```
make ARCH=mips CROSS_COMPILE=mipsel-linux-gnu- wiwa_hd95_defconfig
make ARCH=mips CROSS_COMPILE=mipsel-linux-gnu- -j6      # GCC 12.2, GCC 15 fails
cp vmlinux vmlinux_dtb
mipsel-linux-gnu-objcopy --update-section .appended_dtb=arch/mips/boot/dts/mstar/msd7816-test.dtb vmlinux_dtb
mipsel-linux-gnu-objcopy -O binary vmlinux_dtb vmlinux.bin
readelf -h vmlinux | grep Entry                         # address for MBoot `go`
```
```
Linux kernel
============

There are several guides for kernel developers and users. These guides can
be rendered in a number of formats, like HTML and PDF. Please read
Documentation/admin-guide/README.rst first.

In order to build the documentation, use ``make htmldocs`` or
``make pdfdocs``.  The formatted documentation can also be read online at:

    https://www.kernel.org/doc/html/latest/

There are various text files in the Documentation/ subdirectory,
several of them using the Restructured Text markup notation.

Please read the Documentation/process/changes.rst file, as it contains the
requirements for building and running the kernel, and information about
the problems which may result by upgrading your kernel.
```
