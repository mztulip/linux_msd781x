// SPDX-License-Identifier: GPL-2.0
/*
 * Front panel of MStar MSD7816/MSD7818 (Kronus) set-top boxes: an FD650/TM1650
 * style 2-wire LED/key controller bit-banged on two PM SAR pins used as GPIO
 * (DIO = SAR pin 1, CLK = SAR pin 2 on the WIWA HD95).
 *
 * Keys are polled every 100 ms (like the stock firmware) and reported as an
 * input device. The 4-digit 7-segment display is driven through sysfs:
 *
 *   /sys/.../text        up to 4 characters, '.' lights the dot of the previous one
 *   /sys/.../brightness  0 = display off, 1..7
 *
 * Protocol and pinout derived from the stock firmware, see
 * docs/przyciski_sar.md in the reverse engineering notes.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define DRIVER_NAME		"mstar-fd650-panel"

/* offsets from the node's reg base (PM SAR bank + 0x40) */
#define REG_ANA			0x04	/* 0x1422: 1 = SAR pin in analog mode */
#define REG_OEN			0x05	/* 0x1423: 1 = input */
#define REG_OUT			0x08	/* 0x1424 */
#define REG_IN			0x09	/* 0x1425 */

#define PIN_DIO			BIT(1)
#define PIN_CLK			BIT(2)

#define FD650_CMD_SYSTEM	0x48
#define FD650_SYS_ON		0x01
#define FD650_CMD_READ_KEY	0x4f
#define FD650_KEY_PRESSED	BIT(6)
#define FD650_KEY_ID_MASK	0x3f

#define SEG_DOT			0x20
#define NUM_DIGITS		4
#define POLL_MS			100
#define BIT_DELAY_US		10

/* digit registers, leftmost first */
static const u8 digit_reg[NUM_DIGITS] = { 0x6e, 0x6c, 0x6a, 0x68 };

/* 7-segment font of this board (copied from the stock firmware) */
static const struct {
	char c;
	u8 seg;
} font[] = {
	{ '0', 0xd7 }, { '1', 0x14 }, { '2', 0xcd }, { '3', 0x5d }, { '4', 0x1e },
	{ '5', 0x5b }, { '6', 0xdb }, { '7', 0x15 }, { '8', 0xdf }, { '9', 0x5f },
	{ 'A', 0x9f }, { 'B', 0xda }, { 'b', 0xda }, { 'C', 0xc3 }, { 'c', 0xc8 },
	{ 'D', 0xdc }, { 'd', 0xdc }, { 'E', 0xcb }, { 'F', 0x8b }, { 'H', 0x9e },
	{ 'h', 0x9a }, { 'I', 0x82 }, { 'i', 0x80 }, { 'L', 0xc2 }, { 'n', 0x98 },
	{ 'N', 0x97 }, { 'O', 0xd8 }, { 'o', 0xd8 }, { 'P', 0x8f }, { 'R', 0xc8 },
	{ 'r', 0xc8 }, { 'S', 0x5b }, { 'T', 0xca }, { 't', 0xca }, { 'U', 0xd6 },
	{ 'V', 0xd0 }, { '-', 0x08 }, { ' ', 0x00 },
};

/* key id (code without the pressed bit) -> Linux key code */
static const struct {
	u8 id;
	unsigned short code;
} keymap[] = {
	{ 0x17, KEY_POWER },
	{ 0x0f, KEY_CHANNELUP },
	{ 0x07, KEY_CHANNELDOWN },
};

struct fd650_panel {
	struct device *dev;
	void __iomem *base;
	struct input_dev *input;
	struct mutex lock;	/* serialises bus access */
	u8 seg[NUM_DIGITS];
	char text[NUM_DIGITS * 2 + 1];
	unsigned int brightness;
	unsigned short keycodes[ARRAY_SIZE(keymap)];
	int pressed;		/* index into keymap, -1 = none */
};

/* ---- bit-banged FD650 bus ---- */

static void pin_set(struct fd650_panel *p, u8 pin, bool high)
{
	u8 out = readb(p->base + REG_OUT);

	writeb(high ? out | pin : out & ~pin, p->base + REG_OUT);
	writeb(readb(p->base + REG_OEN) & ~pin, p->base + REG_OEN);
	udelay(BIT_DELAY_US);
}

static bool dio_read(struct fd650_panel *p)
{
	writeb(readb(p->base + REG_OEN) | PIN_DIO, p->base + REG_OEN);
	return readb(p->base + REG_IN) & PIN_DIO;
}

static void bus_start(struct fd650_panel *p)
{
	pin_set(p, PIN_DIO, true);
	pin_set(p, PIN_CLK, true);
	pin_set(p, PIN_DIO, false);
	pin_set(p, PIN_CLK, false);
}

static void bus_stop(struct fd650_panel *p)
{
	pin_set(p, PIN_DIO, false);
	pin_set(p, PIN_CLK, true);
	pin_set(p, PIN_DIO, true);
}

static bool bus_send(struct fd650_panel *p, u8 b)
{
	bool ack;
	int i;

	for (i = 0; i < 8; i++, b <<= 1) {
		pin_set(p, PIN_DIO, b & 0x80);
		pin_set(p, PIN_CLK, true);
		pin_set(p, PIN_CLK, false);
	}
	ack = !dio_read(p);
	udelay(BIT_DELAY_US);
	pin_set(p, PIN_CLK, true);
	pin_set(p, PIN_CLK, false);
	return ack;
}

static u8 bus_recv(struct fd650_panel *p)
{
	u8 v = 0;
	int i;

	dio_read(p);
	for (i = 0; i < 8; i++) {
		udelay(BIT_DELAY_US);
		pin_set(p, PIN_CLK, true);
		v = (v << 1) | dio_read(p);
		pin_set(p, PIN_CLK, false);
	}
	pin_set(p, PIN_DIO, false);	/* master ACK */
	pin_set(p, PIN_CLK, true);
	pin_set(p, PIN_CLK, false);
	return v;
}

static int fd650_cmd(struct fd650_panel *p, u8 cmd, u8 data)
{
	bool ack;

	bus_start(p);
	ack = bus_send(p, cmd) && bus_send(p, data);
	bus_stop(p);
	return ack ? 0 : -EIO;
}

static int fd650_read_key(struct fd650_panel *p, u8 *code)
{
	bool ack;

	bus_start(p);
	ack = bus_send(p, FD650_CMD_READ_KEY);
	*code = bus_recv(p);
	bus_stop(p);
	return ack ? 0 : -EIO;
}

/* ---- display ---- */

static int fd650_update_display(struct fd650_panel *p)
{
	u8 sys = p->brightness ? ((p->brightness & 7) << 4) | FD650_SYS_ON : 0;
	int i, ret;

	/* keep the chip on (key scanning) even when the display is "off" */
	ret = fd650_cmd(p, FD650_CMD_SYSTEM, sys ? sys : (1 << 4) | FD650_SYS_ON);
	for (i = 0; i < NUM_DIGITS && !ret; i++)
		ret = fd650_cmd(p, digit_reg[i], p->brightness ? p->seg[i] : 0);
	return ret;
}

static u8 glyph(char c)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(font); i++)
		if (font[i].c == c)
			return font[i].seg;
	return 0x08;	/* unknown character: '-' */
}

static ssize_t text_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fd650_panel *p = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", p->text);
}

static ssize_t text_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct fd650_panel *p = dev_get_drvdata(dev);
	u8 seg[NUM_DIGITS] = { 0 };
	size_t i;
	int d = 0, ret;

	for (i = 0; i < count && buf[i] != '\n'; i++) {
		if (buf[i] == '.' && d > 0)
			seg[d - 1] |= SEG_DOT;
		else if (d < NUM_DIGITS)
			seg[d++] = glyph(buf[i]);
	}

	mutex_lock(&p->lock);
	memcpy(p->seg, seg, sizeof(seg));
	strscpy(p->text, buf, min(count + 1, sizeof(p->text)));
	strim(p->text);
	ret = fd650_update_display(p);
	mutex_unlock(&p->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(text);

static ssize_t brightness_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fd650_panel *p = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", p->brightness);
}

static ssize_t brightness_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct fd650_panel *p = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;
	if (val > 7)
		return -EINVAL;

	mutex_lock(&p->lock);
	p->brightness = val;
	ret = fd650_update_display(p);
	mutex_unlock(&p->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(brightness);

static struct attribute *fd650_attrs[] = {
	&dev_attr_text.attr,
	&dev_attr_brightness.attr,
	NULL
};
ATTRIBUTE_GROUPS(fd650);

/* ---- keys ---- */

static void fd650_poll(struct input_dev *input)
{
	struct fd650_panel *p = input_get_drvdata(input);
	int idx = -1, i;
	u8 code;

	mutex_lock(&p->lock);
	if (fd650_read_key(p, &code)) {
		mutex_unlock(&p->lock);
		return;
	}
	mutex_unlock(&p->lock);

	if (code & FD650_KEY_PRESSED)
		for (i = 0; i < ARRAY_SIZE(keymap); i++)
			if (keymap[i].id == (code & FD650_KEY_ID_MASK))
				idx = i;

	if (idx == p->pressed)
		return;
	if (p->pressed >= 0)
		input_report_key(input, p->keycodes[p->pressed], 0);
	if (idx >= 0)
		input_report_key(input, p->keycodes[idx], 1);
	input_sync(input);
	p->pressed = idx;
}

static int fd650_probe(struct platform_device *pdev)
{
	struct fd650_panel *p;
	struct input_dev *input;
	int i, ret;

	p = devm_kzalloc(&pdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->dev = &pdev->dev;
	p->pressed = -1;
	p->brightness = 1;
	mutex_init(&p->lock);
	platform_set_drvdata(pdev, p);

	p->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->base))
		return PTR_ERR(p->base);

	/* SAR pins used as digital GPIO, both lines idle high */
	writeb(readb(p->base + REG_ANA) & ~(PIN_DIO | PIN_CLK), p->base + REG_ANA);
	pin_set(p, PIN_DIO, true);
	pin_set(p, PIN_CLK, true);

	mutex_lock(&p->lock);
	ret = fd650_update_display(p);
	mutex_unlock(&p->lock);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "no ACK from panel controller\n");

	input = devm_input_allocate_device(&pdev->dev);
	if (!input)
		return -ENOMEM;

	input->name = "Front panel keys";
	input->phys = DRIVER_NAME "/input0";
	input->id.bustype = BUS_HOST;
	input->keycode = p->keycodes;
	input->keycodesize = sizeof(p->keycodes[0]);
	input->keycodemax = ARRAY_SIZE(p->keycodes);
	for (i = 0; i < ARRAY_SIZE(keymap); i++) {
		p->keycodes[i] = keymap[i].code;
		input_set_capability(input, EV_KEY, p->keycodes[i]);
	}
	input_set_drvdata(input, p);
	p->input = input;

	ret = input_setup_polling(input, fd650_poll);
	if (ret)
		return ret;
	input_set_poll_interval(input, POLL_MS);

	ret = input_register_device(input);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "FD650 front panel ready\n");
	return 0;
}

static const struct of_device_id fd650_of_match[] = {
	{ .compatible = "mstar,msd7816-fd650-panel" },
	{ }
};
MODULE_DEVICE_TABLE(of, fd650_of_match);

static struct platform_driver fd650_driver = {
	.probe = fd650_probe,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = fd650_of_match,
		.dev_groups = fd650_groups,
	},
};
module_platform_driver(fd650_driver);

MODULE_DESCRIPTION("MStar Kronus FD650 front panel (keys + 7-segment display)");
MODULE_LICENSE("GPL");
