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
 *   - FBIOPAN_DISPLAY: remembers the new buffer address; the vsync interrupt
 *     handler writes it (the call does not wait),
 *   - FBIO_WAITFORVSYNC: sleeps until a pending flip is done, or until the
 *     next vsync when nothing is pending.
 *
 * GOP registers are banked: reg 0x1FFE selects the bank. GOP3 uses bank 0xA
 * for its window (GWIN) and bank 0x9 for control.
 *
 * Flipping works like MStar's GFLIP driver (libapiGOP: _MDrv_GFLIP_IntHandler,
 * MHal_GFLIP_SetFlipToGop): on the GOP vsync interrupt (intc IRQ 20, status
 * bit 8 of bank 9 reg 0x1F10, enabled by bits [1:0] = 2) the address is
 * written and applied at once with the force-write bit (0x1FFE bit 9), so it
 * always changes right after vsync. Latching it with the trigger bit (10)
 * at an arbitrary moment of the frame gave occasional shifted/garbage frames
 * under memory load (fbDOOM). See docs/grafika_gop.md and
 * docs/sterownik_gopfb.md in the reverse engineering notes.
 *
 * Memory arbitration: with the MIU defaults the GOP loses against the CPU
 * when the CPU does scattered writes (a game drawing columns), its read
 * FIFO runs dry and HDMI shows shifted lines / garbage for a frame even
 * though the framebuffer content is fine. If the node has a second reg
 * range (MIU0 regs 0x1200-0x12FF), the driver gives MIU client group 1
 * (the GOP clients, as in MStar's clientTbl) high priority and puts that
 * group first in the group arbitration order (what MHal_MIU_SetHPriorityMask
 * and HAL_MIU_SetGroupPriority do).
 */

#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

#define DRIVER_NAME		"mstar-gopfb"

/* offsets from the node's reg base (RIU 0x1F203E00, regs 0x1F00-0x1FFF) */
#define GOP_GWIN_CTRL		0x000	/* bank A 0x1F00: bit 0 enable, [7:4] format */
#define GOP_GWIN_ADDR_L		0x004	/* bank A 0x1F02: buffer address / 8, low */
#define GOP_GWIN_ADDR_H		0x008	/* bank A 0x1F04: high */
#define GOP_GWIN_PITCH		0x024	/* bank A 0x1F12: bytes / 8 */
#define GOP_INT			0x020	/* bank 9 0x1F10: [1:0] int masks, bit 8 vsync status */
#define GOP_BANKSEL		0x1fc	/* 0x1FFE */

#define GOP_FMT_SHIFT		4
#define GOP_FMT_MASK		(0xf << GOP_FMT_SHIFT)
#define GOP_FMT_RGB565		1
#define GOP_FMT_ARGB4444	2

#define BANK_IDLE		0x7000	/* upper bits as left by MBoot */
#define BANK_GOP3_CTRL		0x9
#define BANK_GOP3_GWIN		0xa
#define BANK_FORCE_WRITE	BIT(9)	/* writes take effect immediately */
#define BANK_TRIGGER		BIT(10)
#define BANK_ACK_GOP3		BIT(15)	/* 1 = waiting for vsync */

#define INT_MASK		0x3
#define INT_VSYNC_ON		0x2	/* values from MHal_GFLIP_IntEnable */
#define INT_VSYNC_OFF		0x3
#define INT_VSYNC_STATUS	BIT(8)

/* offsets from the second reg base (RIU 0x1F202400 = MIU0 reg 0x1200) */
#define MIU_GROUP_HPMASK(g)	((0x48 + (g) * 0x20) * 2)	/* 1 = no high priority */
#define MIU_GROUP_ORDER		(0xfe * 2)	/* [7:0] 4 x 2 bit, bit 8 applies */
#define MIU_GOP_GROUP		1

#define VSYNC_TIMEOUT_US	100000	/* 50 Hz output: 20 ms period */

static unsigned int buffers = 2;
module_param(buffers, uint, 0444);
MODULE_PARM_DESC(buffers, "number of 1920x1080 buffers (limited by the memory region)");

struct gopfb {
	void __iomem *regs;
	phys_addr_t base;
	spinlock_t lock;		/* bank select, flip state */
	u32 flip_addr;			/* address / 8 to show at the next vsync */
	bool flip_pending;
	unsigned int vsync_count;
	wait_queue_head_t vsync_wait;
	u32 pseudo_palette[16];
};

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

/* GOP vsync interrupt: apply a pending flip, clear the status, wake waiters */
static irqreturn_t gopfb_irq(int irq, void *data)
{
	struct gopfb *g = data;
	u16 v;

	spin_lock(&g->lock);
	writew(BANK_IDLE | BANK_GOP3_CTRL, g->regs + GOP_BANKSEL);
	v = readw(g->regs + GOP_INT);
	if (!(v & INT_VSYNC_STATUS)) {
		writew(BANK_IDLE, g->regs + GOP_BANKSEL);
		spin_unlock(&g->lock);
		return IRQ_NONE;
	}

	if (g->flip_pending) {
		/* like MHal_GFLIP_SetFlipToGop: write, then force it in now */
		writew(BANK_IDLE | BANK_GOP3_GWIN, g->regs + GOP_BANKSEL);
		writew(g->flip_addr & 0xffff, g->regs + GOP_GWIN_ADDR_L);
		writew(g->flip_addr >> 16, g->regs + GOP_GWIN_ADDR_H);
		writew(BANK_IDLE | BANK_FORCE_WRITE | BANK_GOP3_GWIN, g->regs + GOP_BANKSEL);
		writew(BANK_IDLE | BANK_GOP3_GWIN, g->regs + GOP_BANKSEL);
		g->flip_pending = false;
		writew(BANK_IDLE | BANK_GOP3_CTRL, g->regs + GOP_BANKSEL);
	}

	/* clear the status by masking and unmasking (_MDrv_GFLIP_IntHandler) */
	writew((v & ~INT_MASK) | INT_VSYNC_OFF, g->regs + GOP_INT);
	writew((v & ~INT_MASK) | INT_VSYNC_ON, g->regs + GOP_INT);
	writew(BANK_IDLE, g->regs + GOP_BANKSEL);
	g->vsync_count++;
	spin_unlock(&g->lock);

	wake_up_all(&g->vsync_wait);
	return IRQ_HANDLED;
}

/* give the display (GOP) memory requests priority over the CPU */
static void gopfb_miu_priority(struct device *dev, void __iomem *miu)
{
	u16 order = readw(miu + MIU_GROUP_ORDER);
	u16 hp = readw(miu + MIU_GROUP_HPMASK(MIU_GOP_GROUP));

	/* all clients of the group high priority */
	writew(0x0000, miu + MIU_GROUP_HPMASK(MIU_GOP_GROUP));
	/*
	 * group order, 2 bits per slot from the highest priority: was 0xe4
	 * (0, 1, 2, 3), now 0xe1 (1, 0, 2, 3); bit 8 high-low applies it
	 */
	order = (order & ~0xff) | 0xe1;
	writew(order | BIT(8), miu + MIU_GROUP_ORDER);
	writew(order, miu + MIU_GROUP_ORDER);
	dev_info(dev, "MIU: group %d high priority (mask 0x%04x -> 0x0000), order 0x%02x\n",
		 MIU_GOP_GROUP, hp, order & 0xff);
}

static void gopfb_vsync_irq(struct gopfb *g, bool on)
{
	unsigned long flags;
	u16 v;

	spin_lock_irqsave(&g->lock, flags);
	writew(BANK_IDLE | BANK_GOP3_CTRL, g->regs + GOP_BANKSEL);
	v = readw(g->regs + GOP_INT);
	writew((v & ~INT_MASK) | (on ? INT_VSYNC_ON : INT_VSYNC_OFF), g->regs + GOP_INT);
	writew(BANK_IDLE, g->regs + GOP_BANKSEL);
	spin_unlock_irqrestore(&g->lock, flags);
}

static int gopfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct gopfb *g = info->par;
	unsigned long flags;

	if (var->xoffset || var->yoffset + info->var.yres > info->var.yres_virtual)
		return -EINVAL;

	spin_lock_irqsave(&g->lock, flags);
	g->flip_addr = (g->base + var->yoffset * info->fix.line_length) >> 3;
	g->flip_pending = true;		/* the next vsync interrupt shows it */
	spin_unlock_irqrestore(&g->lock, flags);
	return 0;
}

static bool gopfb_flip_done(struct gopfb *g, unsigned int start)
{
	unsigned long flags;
	bool done;

	spin_lock_irqsave(&g->lock, flags);
	done = !g->flip_pending && g->vsync_count != start;
	spin_unlock_irqrestore(&g->lock, flags);
	return done;
}

/*
 * Wait for the pending flip (it happens at the next vsync), or for the next
 * vsync when nothing is pending. After it returns the previous buffer is no
 * longer scanned out and can be drawn into.
 */
static int gopfb_wait_for_vsync(struct gopfb *g)
{
	unsigned int start = READ_ONCE(g->vsync_count);
	long ret;

	ret = wait_event_interruptible_timeout(g->vsync_wait, gopfb_flip_done(g, start),
					       usecs_to_jiffies(VSYNC_TIMEOUT_US));
	if (ret < 0)
		return ret;
	return ret ? 0 : -ETIMEDOUT;
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
	struct resource *res;
	unsigned long flags;
	u16 ctrl;
	int irq, ret;

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
	init_waitqueue_head(&g->vsync_wait);
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
		/* the bank select must not change before the latch is done */
		ret = gopfb_wait_idle_atomic(g);
		writew(BANK_IDLE, g->regs + GOP_BANKSEL);
	}
	spin_unlock_irqrestore(&g->lock, flags);
	if (ret) {
		dev_err(dev, "GOP3 does not answer (no vsync ACK)\n");
		goto err_release;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		/* plain ioremap: the MIU node in common.dtsi covers the same range */
		void __iomem *miu = devm_ioremap(dev, res->start, resource_size(res));

		if (miu)
			gopfb_miu_priority(dev, miu);
	}

	/* vsync interrupt: flips are done there */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto err_release;
	}
	ret = devm_request_irq(dev, irq, gopfb_irq, 0, DRIVER_NAME, g);
	if (ret) {
		dev_err(dev, "cannot get IRQ %d\n", irq);
		goto err_release;
	}
	gopfb_vsync_irq(g, true);
	if (((ctrl & GOP_FMT_MASK) >> GOP_FMT_SHIFT) != GOP_FMT_RGB565)
		dev_warn(dev, "GOP3 was in format %u, switched to RGB565 (use logofix, not osd_create)\n",
			 (ctrl & GOP_FMT_MASK) >> GOP_FMT_SHIFT);

	ret = register_framebuffer(info);
	if (ret)
		goto err_release;
	platform_set_drvdata(pdev, info);

	dev_info(dev, "fb%d: %ux%u RGB565, %u buffers at %pa, vsync IRQ %d\n",
		 info->node, width, height, nbuf, &rmem->base, irq);
	return 0;

err_release:
	framebuffer_release(info);
	return ret;
}

static void gopfb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);

	gopfb_vsync_irq(info->par, false);
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
