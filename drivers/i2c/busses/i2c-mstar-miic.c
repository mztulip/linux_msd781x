// SPDX-License-Identifier: GPL-2.0
/*
 * MStar MSD7816/MSD7818 (Kronus) hardware I2C master ("MIIC").
 *
 * The controller is a Philips SIO1 (8xC552) style byte engine:
 *
 *   0x00  RESET   bit 0: controller reset
 *   0x34  S1DAT   data byte to send / received byte
 *   0x38  S1CON   CR2 | ENS1 | STA | STO | SI | AA | CR1 | CR0
 *   0x39  S1STA   state code (0x08 START, 0x18 SLA+W ACK, ... 0xF8 idle)
 *
 * Each step is started by writing S1CON with SI cleared and finishes when
 * the hardware sets SI again; S1STA then tells what happened.
 *
 * Register layout and behaviour derived from MStar's halHWI2C (K1 BSP) and
 * verified on a WIWA HD95 board (MxL5007T tuner at 0x60 on port 0).
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define DRIVER_NAME "mstar-miic"

#define REG_RESET	0x00
#define REG_S1DAT	0x34
#define REG_S1CON	0x38
#define REG_S1STA	0x39

#define S1CON_CR2	BIT(7)
#define S1CON_ENS1	BIT(6)
#define S1CON_STA	BIT(5)
#define S1CON_STO	BIT(4)
#define S1CON_SI	BIT(3)
#define S1CON_AA	BIT(2)
#define S1CON_CR_MASK	(S1CON_CR2 | GENMASK(1, 0))

/* S1STA codes, master mode */
#define STA_START		0x08
#define STA_REP_START		0x10
#define STA_SLAW_ACK		0x18
#define STA_SLAW_NACK		0x20
#define STA_DATA_TX_ACK		0x28
#define STA_DATA_TX_NACK	0x30
#define STA_ARB_LOST		0x38
#define STA_SLAR_ACK		0x40
#define STA_SLAR_NACK		0x48
#define STA_DATA_RX_ACK		0x50
#define STA_DATA_RX_NACK	0x58
#define STA_IDLE		0xf8

/* chiptop register 0x1E12 (byte offset 0x24 in the chiptop syscon) */
#define CHIPTOP_I2C_MUX		0x24
#define CHIPTOP_I2C_MUX_P0	GENMASK(1, 0)
#define CHIPTOP_I2C_MUX_P1	GENMASK(6, 4)

#define MIIC_TIMEOUT_US		10000

/*
 * How to write the chiptop pad mux register. The syscon regmap does 32-bit
 * accesses; RIU registers are 16 bit wide. Kept selectable while chasing an
 * HDMI picture jitter that appears when this driver is built in.
 */
static bool pinmux_16bit;
module_param(pinmux_16bit, bool, 0444);
MODULE_PARM_DESC(pinmux_16bit, "write the pad mux with a 16-bit access instead of the syscon regmap");

struct mstar_miic {
	struct i2c_adapter adap;
	struct device *dev;
	void __iomem *base;
	u8 cr;		/* clock rate bits for S1CON */
};

static u8 miic_status(struct mstar_miic *miic)
{
	return readb(miic->base + REG_S1STA);
}

/* Start the next bus step by writing S1CON (SI cleared) and wait for SI. */
static int miic_step(struct mstar_miic *miic, u8 ctrl)
{
	u8 con;

	writeb(S1CON_ENS1 | miic->cr | ctrl, miic->base + REG_S1CON);
	return readb_poll_timeout_atomic(miic->base + REG_S1CON, con,
					 con & S1CON_SI, 1, MIIC_TIMEOUT_US);
}

static void miic_stop(struct mstar_miic *miic)
{
	u8 sta;

	writeb(S1CON_ENS1 | miic->cr | S1CON_STO | S1CON_AA,
	       miic->base + REG_S1CON);
	if (readb_poll_timeout_atomic(miic->base + REG_S1STA, sta,
				      sta == STA_IDLE, 1, MIIC_TIMEOUT_US))
		dev_dbg(miic->dev, "stop timeout, status 0x%02x\n", sta);
}

static void miic_reset(struct mstar_miic *miic)
{
	writeb(1, miic->base + REG_RESET);
	writeb(0, miic->base + REG_RESET);
	writeb(S1CON_ENS1 | miic->cr, miic->base + REG_S1CON);
}

static int miic_xfer_msg(struct mstar_miic *miic, struct i2c_msg *msg)
{
	bool rd = msg->flags & I2C_M_RD;
	u8 sta;
	int i, ret;

	ret = miic_step(miic, S1CON_STA | S1CON_AA);
	if (ret)
		return ret;
	sta = miic_status(miic);
	if (sta != STA_START && sta != STA_REP_START)
		return sta == STA_ARB_LOST ? -EAGAIN : -EIO;

	writeb(i2c_8bit_addr_from_msg(msg), miic->base + REG_S1DAT);
	ret = miic_step(miic, S1CON_AA);
	if (ret)
		return ret;
	sta = miic_status(miic);
	if (sta == STA_SLAW_NACK || sta == STA_SLAR_NACK)
		return -ENXIO;
	if (sta != (rd ? STA_SLAR_ACK : STA_SLAW_ACK))
		return sta == STA_ARB_LOST ? -EAGAIN : -EIO;

	for (i = 0; i < msg->len; i++) {
		if (rd) {
			bool last = i == msg->len - 1;

			ret = miic_step(miic, last ? 0 : S1CON_AA);
			if (ret)
				return ret;
			sta = miic_status(miic);
			if (sta != (last ? STA_DATA_RX_NACK : STA_DATA_RX_ACK))
				return -EIO;
			msg->buf[i] = readb(miic->base + REG_S1DAT);
		} else {
			writeb(msg->buf[i], miic->base + REG_S1DAT);
			ret = miic_step(miic, S1CON_AA);
			if (ret)
				return ret;
			sta = miic_status(miic);
			if (sta == STA_DATA_TX_NACK)
				return -EIO;
			if (sta != STA_DATA_TX_ACK)
				return sta == STA_ARB_LOST ? -EAGAIN : -EIO;
		}
	}

	return 0;
}

static int mstar_miic_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			   int num)
{
	struct mstar_miic *miic = i2c_get_adapdata(adap);
	int i, ret = 0;

	for (i = 0; i < num; i++) {
		ret = miic_xfer_msg(miic, &msgs[i]);
		if (ret)
			break;
	}

	miic_stop(miic);
	if (ret == -ETIMEDOUT || ret == -EAGAIN)
		miic_reset(miic);

	return ret ? ret : num;
}

static u32 mstar_miic_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm mstar_miic_algo = {
	.master_xfer = mstar_miic_xfer,
	.functionality = mstar_miic_func,
};

/*
 * The SoC pads are routed to the I2C master by a chiptop register; MBoot only
 * sets it when it initialises the tuner, so do it here.
 */
static int mstar_miic_setup_pinmux(struct mstar_miic *miic)
{
	struct device_node *np = miic->dev->of_node;
	struct regmap *chiptop;
	u32 port, mode;

	if (of_property_read_u32(np, "mstar,pad-mode", &mode))
		return 0;

	if (of_property_read_u32(np, "mstar,port", &port))
		port = 0;

	if (pinmux_16bit) {
		struct device_node *ct = of_parse_phandle(np, "mstar,chiptop", 0);
		void __iomem *reg;
		u16 mask = port ? CHIPTOP_I2C_MUX_P1 : CHIPTOP_I2C_MUX_P0;
		u16 v;

		if (!ct)
			return dev_err_probe(miic->dev, -ENODEV, "no mstar,chiptop\n");
		reg = of_iomap(ct, 0);
		of_node_put(ct);
		if (!reg)
			return -ENOMEM;
		v = readw(reg + CHIPTOP_I2C_MUX) & ~mask;
		v |= port ? FIELD_PREP(CHIPTOP_I2C_MUX_P1, mode) :
			    FIELD_PREP(CHIPTOP_I2C_MUX_P0, mode);
		writew(v, reg + CHIPTOP_I2C_MUX);
		iounmap(reg);
		return 0;
	}

	chiptop = syscon_regmap_lookup_by_phandle(np, "mstar,chiptop");
	if (IS_ERR(chiptop))
		return dev_err_probe(miic->dev, PTR_ERR(chiptop),
				     "no chiptop syscon for pad mode\n");

	if (port == 0)
		return regmap_update_bits(chiptop, CHIPTOP_I2C_MUX,
					  CHIPTOP_I2C_MUX_P0,
					  FIELD_PREP(CHIPTOP_I2C_MUX_P0, mode));

	return regmap_update_bits(chiptop, CHIPTOP_I2C_MUX, CHIPTOP_I2C_MUX_P1,
				  FIELD_PREP(CHIPTOP_I2C_MUX_P1, mode));
}

static int mstar_miic_probe(struct platform_device *pdev)
{
	struct mstar_miic *miic;
	u32 cr = 1;
	int ret;

	miic = devm_kzalloc(&pdev->dev, sizeof(*miic), GFP_KERNEL);
	if (!miic)
		return -ENOMEM;

	miic->dev = &pdev->dev;
	miic->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(miic->base))
		return PTR_ERR(miic->base);

	/* Raw S1CON clock-rate bits; the bus frequency for each is unmeasured. */
	of_property_read_u32(pdev->dev.of_node, "mstar,clock-rate", &cr);
	miic->cr = cr & S1CON_CR_MASK;

	ret = mstar_miic_setup_pinmux(miic);
	if (ret)
		return ret;

	miic_reset(miic);

	miic->adap.owner = THIS_MODULE;
	miic->adap.algo = &mstar_miic_algo;
	miic->adap.dev.parent = &pdev->dev;
	miic->adap.dev.of_node = pdev->dev.of_node;
	strscpy(miic->adap.name, dev_name(&pdev->dev), sizeof(miic->adap.name));
	i2c_set_adapdata(&miic->adap, miic);
	platform_set_drvdata(pdev, miic);

	return devm_i2c_add_adapter(&pdev->dev, &miic->adap);
}

static const struct of_device_id mstar_miic_of_match[] = {
	{ .compatible = "mstar,msd7816-miic" },
	{ }
};
MODULE_DEVICE_TABLE(of, mstar_miic_of_match);

static struct platform_driver mstar_miic_driver = {
	.probe = mstar_miic_probe,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = mstar_miic_of_match,
	},
};
module_platform_driver(mstar_miic_driver);

MODULE_DESCRIPTION("MStar MSD7816/Kronus hardware I2C master");
MODULE_LICENSE("GPL");
