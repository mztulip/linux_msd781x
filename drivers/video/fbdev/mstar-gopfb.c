// SPDX-License-Identifier: GPL-2.0
/*
 * Framebuffer on the GOP3 graphics layer of MStar Kronus (MSD7816/MSD7818)
 * set-top-box SoCs, with double buffering.
 *
 * MBoot (plus the logofix helper) leaves GOP3 showing a 1920x1080 RGB565
 * window. The output timing (HDMI) is kept as set by the bootloader. This
 * driver takes the layer over and adds:
 *
 *   - yres_virtual = N * yres, with N buffers in the reserved memory region,
 *   - FBIOPAN_DISPLAY: the new buffer address is latched at the next vsync
 *     (the call does not wait for it),
 *   - FBIO_WAITFORVSYNC: waits until a pending flip is done, or for the next
 *     vsync when nothing is pending.
 *
 * GOP registers are banked: reg 0x1FFE selects the bank. GOP3 uses bank 0xA
 * for its window (GWIN) and bank 0x9 for control. Writing the control bank
 * with bit 10 (trigger) set latches the pending GOP3 registers at the next
 * vsync; bit 15 stays 1 until that happens, and changing the bank select
 * before it drops cancels the update. See docs/grafika_gop.md in the reverse
 * engineering notes.
 */

#include <linux/fb.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define DRIVER_NAME		"mstar-gopfb"

/* offsets from the node's reg base (RIU 0x1F203E00, regs 0x1F00-0x1FFF) */
#define GOP_GWIN_CTRL		0x000	/* bank A 0x1F00: bit 0 enable, [7:4] format */
#define GOP_GWIN_ADDR_L		0x004	/* bank A 0x1F02: buffer address / 8, low */
#define GOP_GWIN_ADDR_H		0x008	/* bank A 0x1F04: high */
#define GOP_GWIN_PITCH		0x024	/* bank A 0x1F12: bytes / 8 */
#define GOP_BANKSEL		0x1fc	/* 0x1FFE */

#define GOP_FMT_SHIFT		4
#define GOP_FMT_MASK		(0xf << GOP_FMT_SHIFT)
#define GOP_FMT_RGB565		1
#define GOP_FMT_ARGB4444	2

#define BANK_IDLE		0x7000	/* upper bits as left by MBoot */
#define BANK_GOP3_CTRL		0x9
#define BANK_GOP3_GWIN		0xa
#define BANK_TRIGGER		BIT(10)
#define BANK_ACK_GOP3		BIT(15)	/* 1 = waiting for vsync */

#define VSYNC_TIMEOUT_US	100000	/* 50 Hz output: 20 ms period */

static unsigned int buffers = 2;
module_param(buffers, uint, 0444);
MODULE_PARM_DESC(buffers, "number of 1920x1080 buffers (limited by the memory region)");

struct gopfb {
	void __iomem *regs;
	phys_addr_t base;
	spinlock_t lock;		/* bank select and the trigger sequence */
	u32 pseudo_palette[16];
};

static bool gopfb_pending(struct gopfb *g)
{
	return readw(g->regs + GOP_BANKSEL) & BANK_ACK_GOP3;
}

/* wait until the previous latch is done; caller holds g->lock */
static int gopfb_wait_idle_atomic(struct gopfb *g)
{
	u16 v;

	return readw_poll_timeout_atomic(g->regs + GOP_BANKSEL, v,
					 !(v & BANK_ACK_GOP3), 10, VSYNC_TIMEOUT_US);
}

/* latch the pending GOP3 registers at the next vsync; caller holds g->lock */
static void gopfb_trigger(struct gopfb *g)
{
	writew(BANK_IDLE | BANK_TRIGGER | BANK_GOP3_CTRL, g->regs + GOP_BANKSEL);
	writew(BANK_IDLE | BANK_GOP3_CTRL, g->regs + GOP_BANKSEL);
}

static int gopfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct gopfb *g = info->par;
	u32 addr = (g->base + var->yoffset * info->fix.line_length) >> 3;
	unsigned long flags;
	int ret;

	if (var->xoffset || var->yoffset + info->var.yres > info->var.yres_virtual)
		return -EINVAL;

	spin_lock_irqsave(&g->lock, flags);
	ret = gopfb_wait_idle_atomic(g);
	if (!ret) {
		writew(BANK_IDLE | BANK_GOP3_GWIN, g->regs + GOP_BANKSEL);
		writew(addr & 0xffff, g->regs + GOP_GWIN_ADDR_L);
		writew(addr >> 16, g->regs + GOP_GWIN_ADDR_H);
		gopfb_trigger(g);	/* bank select stays on 9 until the ACK */
	}
	spin_unlock_irqrestore(&g->lock, flags);
	return ret;
}

static int gopfb_wait_for_vsync(struct gopfb *g)
{
	unsigned long flags;
	u16 v;

	spin_lock_irqsave(&g->lock, flags);
	if (!gopfb_pending(g))
		gopfb_trigger(g);	/* nothing pending: a latch is a vsync wait */
	spin_unlock_irqrestore(&g->lock, flags);

	return readw_poll_timeout(g->regs + GOP_BANKSEL, v, !(v & BANK_ACK_GOP3),
				  500, VSYNC_TIMEOUT_US);
}

static int gopfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		return gopfb_wait_for_vsync(info->par);
	default:
		return -ENOTTY;
	}
}

static int gopfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			   u_int transp, struct fb_info *info)
{
	u32 *pal = info->pseudo_palette;

	if (regno >= 16)
		return -EINVAL;
	pal[regno] = (red >> (16 - info->var.red.length)) << info->var.red.offset |
		     (green >> (16 - info->var.green.length)) << info->var.green.offset |
		     (blue >> (16 - info->var.blue.length)) << info->var.blue.offset;
	return 0;
}

static int gopfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	/* fixed mode, only the panning offset may change */
	if (var->xres != info->var.xres || var->yres != info->var.yres ||
	    var->xres_virtual != info->var.xres_virtual ||
	    var->yres_virtual != info->var.yres_virtual ||
	    var->bits_per_pixel != info->var.bits_per_pixel)
		return -EINVAL;
	return 0;
}

static const struct fb_ops gopfb_ops = {
	.owner		= THIS_MODULE,
	FB_DEFAULT_IOMEM_OPS,
	.fb_check_var	= gopfb_check_var,
	.fb_pan_display	= gopfb_pan_display,
	.fb_ioctl	= gopfb_ioctl,
	.fb_setcolreg	= gopfb_setcolreg,
};

static int gopfb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reserved_mem *rmem;
	struct device_node *np;
	struct fb_info *info;
	struct gopfb *g;
	u32 width, height, pitch, frame, nbuf;
	unsigned long flags;
	u16 ctrl;
	int ret;

	if (of_property_read_u32(dev->of_node, "width", &width) ||
	    of_property_read_u32(dev->of_node, "height", &height))
		return dev_err_probe(dev, -EINVAL, "width/height missing\n");
	pitch = width * 2;
	frame = pitch * height;

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "memory-region missing\n");
	rmem = of_reserved_mem_lookup(np);
	of_node_put(np);
	if (!rmem)
		return dev_err_probe(dev, -EINVAL, "memory-region not reserved\n");

	nbuf = min_t(u32, buffers, rmem->size / frame);
	if (!nbuf)
		return dev_err_probe(dev, -ENOMEM, "memory-region too small\n");

	info = framebuffer_alloc(sizeof(*g), dev);
	if (!info)
		return -ENOMEM;
	g = info->par;
	spin_lock_init(&g->lock);
	g->base = rmem->base;

	g->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(g->regs)) {
		ret = PTR_ERR(g->regs);
		goto err_release;
	}

	info->screen_base = devm_ioremap(dev, rmem->base, frame * nbuf);
	if (!info->screen_base) {
		ret = -ENOMEM;
		goto err_release;
	}

	strscpy(info->fix.id, "mstar-gop3", sizeof(info->fix.id));
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.accel = FB_ACCEL_NONE;
	info->fix.smem_start = rmem->base;
	info->fix.smem_len = frame * nbuf;
	info->fix.line_length = pitch;
	info->fix.ypanstep = 1;

	info->var.xres = info->var.xres_virtual = width;
	info->var.yres = height;
	info->var.yres_virtual = height * nbuf;
	info->var.bits_per_pixel = 16;
	info->var.red = (struct fb_bitfield){ 11, 5, 0 };
	info->var.green = (struct fb_bitfield){ 5, 6, 0 };
	info->var.blue = (struct fb_bitfield){ 0, 5, 0 };
	info->var.activate = FB_ACTIVATE_NOW;
	info->var.vmode = FB_VMODE_NONINTERLACED;
	info->var.height = info->var.width = -1;

	info->fbops = &gopfb_ops;
	info->pseudo_palette = g->pseudo_palette;
	info->screen_size = info->fix.smem_len;
	/* no FBINFO_HWACCEL_YPAN: fbcon redraws instead of panning */
	info->flags = 0;

	/* show buffer 0 in RGB565 (osd_create would have left ARGB4444) */
	spin_lock_irqsave(&g->lock, flags);
	ret = gopfb_wait_idle_atomic(g);
	if (!ret) {
		u32 addr = g->base >> 3;

		writew(BANK_IDLE | BANK_GOP3_GWIN, g->regs + GOP_BANKSEL);
		ctrl = readw(g->regs + GOP_GWIN_CTRL);
		writew((ctrl & ~GOP_FMT_MASK) | GOP_FMT_RGB565 << GOP_FMT_SHIFT,
		       g->regs + GOP_GWIN_CTRL);
		writew(addr & 0xffff, g->regs + GOP_GWIN_ADDR_L);
		writew(addr >> 16, g->regs + GOP_GWIN_ADDR_H);
		writew(pitch >> 3, g->regs + GOP_GWIN_PITCH);
		gopfb_trigger(g);
	}
	spin_unlock_irqrestore(&g->lock, flags);
	if (ret) {
		dev_err(dev, "GOP3 does not answer (no vsync ACK)\n");
		goto err_release;
	}
	if (((ctrl & GOP_FMT_MASK) >> GOP_FMT_SHIFT) != GOP_FMT_RGB565)
		dev_warn(dev, "GOP3 was in format %u, switched to RGB565 (use logofix, not osd_create)\n",
			 (ctrl & GOP_FMT_MASK) >> GOP_FMT_SHIFT);

	ret = register_framebuffer(info);
	if (ret)
		goto err_release;
	platform_set_drvdata(pdev, info);

	dev_info(dev, "fb%d: %ux%u RGB565, %u buffers at %pa\n",
		 info->node, width, height, nbuf, &rmem->base);
	return 0;

err_release:
	framebuffer_release(info);
	return ret;
}

static void gopfb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);

	unregister_framebuffer(info);
	framebuffer_release(info);
}

static const struct of_device_id gopfb_of_match[] = {
	{ .compatible = "mstar,msd7816-gop3-fb" },
	{ }
};
MODULE_DEVICE_TABLE(of, gopfb_of_match);

static struct platform_driver gopfb_driver = {
	.probe = gopfb_probe,
	.remove_new = gopfb_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = gopfb_of_match,
	},
};
module_platform_driver(gopfb_driver);

MODULE_DESCRIPTION("MStar Kronus GOP3 framebuffer with double buffering");
MODULE_LICENSE("GPL");
