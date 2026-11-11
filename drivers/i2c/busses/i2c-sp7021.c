// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sunplus SP7021 I2C master driver
 *
 * Copyright (c) Sunplus Inc.
 * Author: Li-hao Kuo <lhjeff911@gmail.com>
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/kmemleak.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/units.h>

#define SP_I2C_BITS_PER_BYTE			9
#define SP_I2C_SW_RESET_DELAY_US		2
#define SP_I2C_SCL_DELAY			1
#define SP_I2C_SOURCE_CLK_HZ			(27 * HZ_PER_MHZ)

#define SP_I2C_DMA_THRESHOLD_BYTES		16
#define SP_I2C_DATA_REG_BYTES			4
#define SP_I2C_FIFO_BYTES			32

#define SP_I2C_CTL0_REG				0x0000
#define SP_I2C_CTL0_FREQ_MASK			GENMASK(26, 24)
#define SP_I2C_CTL0_PREFETCH			BIT(18)
#define SP_I2C_CTL0_RESTART_EN			BIT(17)
#define SP_I2C_CTL0_SUBADDR_EN			BIT(16)
#define SP_I2C_CTL0_SW_RESET			BIT(15)
#define SP_I2C_CTL0_SLAVE_ADDR_MASK		GENMASK(7, 1)

#define SP_I2C_CTL1_REG				0x0004
#define SP_I2C_CTL1_ALL_CLR			GENMASK(9, 0)
#define SP_I2C_CTL1_SCL_HOLD_TOO_LONG_CLR	BIT(8)
#define SP_I2C_CTL1_DATA_NACK_CLR		BIT(5)
#define SP_I2C_CTL1_ADDR_NACK_CLR		BIT(4)
#define SP_I2C_CTL1_DONE_CLR			BIT(1)

#define SP_I2C_CTL2_REG				0x0008
#define SP_I2C_CTL2_FREQ_CUSTOM_MASK		GENMASK(10, 0)
#define SP_I2C_CTL2_SCL_DELAY_MASK		GENMASK(25, 24)
#define SP_I2C_CTL2_SDA_HALF_EN			BIT(31)

#define SP_I2C_INT_REG				0x001c
#define SP_I2C_INT_EN0_REG			0x0020
#define SP_I2C_MOD_REG				0x0024
#define SP_I2C_INT_EN1_REG			0x0034
#define SP_I2C_INT_EN2_REG			0x0040
#define SP_I2C_CTL7_REG				0x0044
#define SP_I2C_CTL7_RD_MASK			GENMASK(31, 16)
#define SP_I2C_CTL7_WR_MASK			GENMASK(15, 0)
#define SP_I2C_MODE_MASK			GENMASK(2, 0)
#define SP_I2C_MODE_DMA_MODE			BIT(2)
#define SP_I2C_MODE_MANUAL_MODE			BIT(1)
#define SP_I2C_MODE_MANUAL_TRIG			BIT(0)

#define SP_I2C_DATA_REG(n)			(0x0060 + (n) * SP_I2C_DATA_REG_BYTES)

#define SP_I2C_DMA_CONF_REG			0x0084
#define SP_I2C_DMA_LEN_REG			0x0088
#define SP_I2C_DMA_LEN_MASK			GENMASK(15, 0)
#define SP_I2C_DMA_ADDR_REG			0x008c
#define SP_I2C_DMA_CFG_DMA_GO			BIT(8)
#define SP_I2C_DMA_CFG_DMA_MODE			BIT(0)

#define SP_I2C_DMA_FLAG_REG			0x0094
#define SP_I2C_DMA_INT_LEN0_FLG			BIT(6)
#define SP_I2C_DMA_INT_IP_TMO_FLG		BIT(4)
#define SP_I2C_DMA_INT_GDMA_TMO_FLG		BIT(3)
#define SP_I2C_DMA_INT_WB_EN_ERR_FLG		BIT(2)
#define SP_I2C_DMA_INT_WCNT_ERR_FLG		BIT(1)
#define SP_I2C_DMA_INT_DMA_DONE_FLG		BIT(0)
#define SP_I2C_DMA_INT_ALL_FLG			GENMASK(6, 0)
#define SP_I2C_DMA_INT_ERR_FLG			(SP_I2C_DMA_INT_LEN0_FLG | \
						 SP_I2C_DMA_INT_IP_TMO_FLG | \
						 SP_I2C_DMA_INT_GDMA_TMO_FLG | \
						 SP_I2C_DMA_INT_WB_EN_ERR_FLG | \
						 SP_I2C_DMA_INT_WCNT_ERR_FLG)

#define SP_I2C_DMA_INT_EN_REG			0x0098
#define SP_I2C_DMA_EN_DMA_DONE_INT		BIT(0)
/* Each DMA error has its enable bit at the position of its flag. */
#define SP_I2C_DMA_EN_ERR_INTS			SP_I2C_DMA_INT_ERR_FLG

#define SP_I2C_DMA_SW_RESET_REG			0x009c
#define SP_I2C_DMA_SW_RESET			BIT(1)
#define SP_I2C_DMA_SW_RESET_DONE		BIT(0)
#define SP_I2C_DMA_SW_RESET_POLL_US		10
#define SP_I2C_DMA_SW_RESET_TIMEOUT_US		1000

#define SP_I2C_INT_SCL_HOLD_TOO_LONG_FLG	BIT(11)
#define SP_I2C_INT_DATA_NACK_FLG		BIT(5)
#define SP_I2C_INT_ADDR_NACK_FLG		BIT(4)
#define SP_I2C_INT_DONE_FLG			BIT(1)

#define SP_I2C_EN0_SCL_HOLD_TOO_LONG_INT	BIT(13)
#define SP_I2C_EN0_DATA_NACK_INT		BIT(5)
#define SP_I2C_EN0_ADDR_NACK_INT		BIT(4)
#define SP_I2C_EN0_DONE_INT			BIT(1)
#define SP_I2C_EN0_TRANSFER_INTS		(SP_I2C_EN0_SCL_HOLD_TOO_LONG_INT | \
						 SP_I2C_EN0_DATA_NACK_INT | \
						 SP_I2C_EN0_ADDR_NACK_INT | \
						 SP_I2C_EN0_DONE_INT)

#define SP_I2C_MIN_CLOCK_FREQ			13184

enum sp_i2c_dma_mode {
	I2C_DMA_WRITE_MODE,
	I2C_DMA_READ_MODE,
};

enum sp_i2c_mode {
	I2C_WRITE_MODE,
	I2C_READ_MODE,
	I2C_RESTART_MODE,
};

struct sp_i2c_dev {
	struct device *dev;
	struct i2c_adapter adap;
	void __iomem *i2c_regs;
	int irq;
	u32 clock_divider;
	u32 bus_freq_hz;
	bool dma_wedged;
	struct completion transfer_done;
	bool transfer_uses_dma;
	bool i2c_done;
	bool dma_done;
	int transfer_error;
};

static unsigned int sp_i2cm_get_int_flag(void __iomem *sr)
{
	return readl(sr + SP_I2C_INT_REG);
}

static void sp_i2cm_status_clear(void __iomem *sr, u32 flag)
{
	u32 ctl1;

	ctl1 = readl(sr + SP_I2C_CTL1_REG);
	ctl1 |= flag;
	writel(ctl1, sr + SP_I2C_CTL1_REG);
	ctl1 = readl(sr + SP_I2C_CTL1_REG);
	ctl1 &= (~flag);
	writel(ctl1, sr + SP_I2C_CTL1_REG);
}

static void sp_i2cm_reset(void __iomem *sr)
{
	u32 ctl0;

	ctl0 = readl(sr + SP_I2C_CTL0_REG);
	ctl0 |= SP_I2C_CTL0_SW_RESET;
	writel(ctl0, sr + SP_I2C_CTL0_REG);
	readl(sr + SP_I2C_CTL0_REG);	/* wait for the write to complete before delaying */
	udelay(SP_I2C_SW_RESET_DELAY_US);
}

static void sp_i2cm_addr_freq_set(void __iomem *sr, u32 divider, unsigned int addr)
{
	u32 ctl0, ctl2;

	ctl0 = readl(sr + SP_I2C_CTL0_REG);
	/* FREQ = 0 selects FREQ_CUSTOM. */
	ctl0 &= ~SP_I2C_CTL0_FREQ_MASK;
	ctl0 &= ~SP_I2C_CTL0_SLAVE_ADDR_MASK;
	ctl0 |= FIELD_PREP(SP_I2C_CTL0_SLAVE_ADDR_MASK, addr);
	writel(ctl0, sr + SP_I2C_CTL0_REG);

	ctl2 = readl(sr + SP_I2C_CTL2_REG);
	ctl2 &= ~SP_I2C_CTL2_FREQ_CUSTOM_MASK;
	ctl2 |= FIELD_PREP(SP_I2C_CTL2_FREQ_CUSTOM_MASK, divider);
	writel(ctl2, sr + SP_I2C_CTL2_REG);
}

static void sp_i2cm_scl_delay_set(void __iomem *sr, unsigned int delay)
{
	u32 ctl2;

	ctl2 = readl(sr + SP_I2C_CTL2_REG);
	ctl2 &= ~SP_I2C_CTL2_SCL_DELAY_MASK;
	ctl2 |= FIELD_PREP(SP_I2C_CTL2_SCL_DELAY_MASK, delay);
	ctl2 &= ~SP_I2C_CTL2_SDA_HALF_EN;
	writel(ctl2, sr + SP_I2C_CTL2_REG);
}

static void sp_i2cm_trans_cnt_set(void __iomem *sr, unsigned int write_cnt,
				  unsigned int read_cnt)
{
	u32 ctl7;

	ctl7 = FIELD_PREP(SP_I2C_CTL7_RD_MASK, read_cnt) |
		FIELD_PREP(SP_I2C_CTL7_WR_MASK, write_cnt);
	writel(ctl7, sr + SP_I2C_CTL7_REG);
}

static void sp_i2cm_active_mode_set(void __iomem *sr, u32 mode)
{
	u32 val;

	val = readl(sr + SP_I2C_MOD_REG);
	val &= ~SP_I2C_MODE_MASK;
	val |= mode;
	writel(val, sr + SP_I2C_MOD_REG);
}

static void sp_i2cm_rw_mode_set(void __iomem *sr, enum sp_i2c_mode rw_mode)
{
	u32 ctl0;

	ctl0 = readl(sr + SP_I2C_CTL0_REG);
	switch (rw_mode) {
	default:
	case I2C_WRITE_MODE:
		ctl0 &= ~(SP_I2C_CTL0_PREFETCH |
			  SP_I2C_CTL0_RESTART_EN | SP_I2C_CTL0_SUBADDR_EN);
		break;
	case I2C_READ_MODE:
		ctl0 &= ~(SP_I2C_CTL0_RESTART_EN | SP_I2C_CTL0_SUBADDR_EN);
		ctl0 |= SP_I2C_CTL0_PREFETCH;
		break;
	case I2C_RESTART_MODE:
		ctl0 |= (SP_I2C_CTL0_PREFETCH |
			 SP_I2C_CTL0_RESTART_EN | SP_I2C_CTL0_SUBADDR_EN);
		break;
	}
	writel(ctl0, sr + SP_I2C_CTL0_REG);
}

static void sp_i2cm_manual_trigger(void __iomem *sr)
{
	u32 val;

	val = readl(sr + SP_I2C_MOD_REG);
	val |= SP_I2C_MODE_MANUAL_TRIG;
	writel(val, sr + SP_I2C_MOD_REG);
}

static unsigned int sp_i2cm_get_dma_int_flag(void __iomem *sr_dma)
{
	return readl(sr_dma + SP_I2C_DMA_FLAG_REG);
}

/* Write 1 to clear: write only the given flags, so a flag set after they were read survives. */
static void sp_i2cm_dma_int_flag_clear(void __iomem *sr_dma, unsigned int flag)
{
	writel(flag, sr_dma + SP_I2C_DMA_FLAG_REG);
}

static void sp_i2cm_dma_addr_set(void __iomem *sr_dma, dma_addr_t addr)
{
	writel(addr, sr_dma + SP_I2C_DMA_ADDR_REG);
}

static void sp_i2cm_dma_len_set(void __iomem *sr_dma, unsigned int len)
{
	writel(FIELD_PREP(SP_I2C_DMA_LEN_MASK, len), sr_dma + SP_I2C_DMA_LEN_REG);
}

static void sp_i2cm_dma_rw_mode_set(void __iomem *sr_dma, enum sp_i2c_dma_mode rw_mode)
{
	u32 val;

	val = readl(sr_dma + SP_I2C_DMA_CONF_REG);
	switch (rw_mode) {
	default:
	case I2C_DMA_WRITE_MODE:
		val |= SP_I2C_DMA_CFG_DMA_MODE;
		break;
	case I2C_DMA_READ_MODE:
		val &= ~SP_I2C_DMA_CFG_DMA_MODE;
		break;
	}
	writel(val, sr_dma + SP_I2C_DMA_CONF_REG);
}

static void sp_i2cm_dma_int_en_set(void __iomem *sr_dma, unsigned int dma_int)
{
	writel(dma_int, sr_dma + SP_I2C_DMA_INT_EN_REG);
}

static void sp_i2cm_dma_go_set(void __iomem *sr_dma)
{
	u32 val;

	val = readl(sr_dma + SP_I2C_DMA_CONF_REG);
	val |= SP_I2C_DMA_CFG_DMA_GO;
	writel(val, sr_dma + SP_I2C_DMA_CONF_REG);
}

static int sp_i2cm_dma_soft_reset(void __iomem *sr_dma)
{
	u32 val;

	/*
	 * SW RESET DONE is a state bit, not a latched flag (data-sheet 141.7):
	 * it goes to 0 as soon as the reset starts, so the first read after the
	 * write cannot see a stale 1. A reset of an idle engine completes
	 * before that read, so waiting for DONE to go to 0 first would fail.
	 */
	writel(SP_I2C_DMA_SW_RESET, sr_dma + SP_I2C_DMA_SW_RESET_REG);
	return readl_poll_timeout(sr_dma + SP_I2C_DMA_SW_RESET_REG, val,
				  val & SP_I2C_DMA_SW_RESET_DONE,
				  SP_I2C_DMA_SW_RESET_POLL_US,
				  SP_I2C_DMA_SW_RESET_TIMEOUT_US);
}

static void sp_i2c_fifo_write(void __iomem *sr, const u8 *buf, unsigned int len)
{
	u8 fifo[SP_I2C_FIFO_BYTES] = {};
	unsigned int i;

	memcpy(fifo, buf, len);
	for (i = 0; i < DIV_ROUND_UP(len, SP_I2C_DATA_REG_BYTES); i++)
		writel(get_unaligned_le32(fifo + i * SP_I2C_DATA_REG_BYTES),
		       sr + SP_I2C_DATA_REG(i));
}

static void sp_i2c_fifo_read(void __iomem *sr, u8 *buf, unsigned int len)
{
	u8 fifo[SP_I2C_FIFO_BYTES];
	unsigned int i;

	for (i = 0; i < DIV_ROUND_UP(len, SP_I2C_DATA_REG_BYTES); i++)
		put_unaligned_le32(readl(sr + SP_I2C_DATA_REG(i)),
				   fifo + i * SP_I2C_DATA_REG_BYTES);
	memcpy(buf, fifo, len);
}

/* The CTL1 clear bits of the handled INT flags; they are not all at the same positions. */
static u32 sp_i2cm_int_flags_to_ctl1_clear(u32 int_flg)
{
	u32 clear = 0;

	if (int_flg & SP_I2C_INT_DONE_FLG)
		clear |= SP_I2C_CTL1_DONE_CLR;
	if (int_flg & SP_I2C_INT_ADDR_NACK_FLG)
		clear |= SP_I2C_CTL1_ADDR_NACK_CLR;
	if (int_flg & SP_I2C_INT_DATA_NACK_FLG)
		clear |= SP_I2C_CTL1_DATA_NACK_CLR;
	if (int_flg & SP_I2C_INT_SCL_HOLD_TOO_LONG_FLG)
		clear |= SP_I2C_CTL1_SCL_HOLD_TOO_LONG_CLR;
	return clear;
}

static irqreturn_t _sp_i2cm_irqevent_handler(int irq, void *args)
{
	struct sp_i2c_dev *spi2c = args;
	void __iomem *sr = spi2c->i2c_regs;
	unsigned int int_flg, dma_flg = 0;
	bool i2c_done;
	int error = 0;

	/* Clear only what was read: an event that latches meanwhile raises the IRQ again. */
	int_flg = sp_i2cm_get_int_flag(sr);
	sp_i2cm_status_clear(sr, sp_i2cm_int_flags_to_ctl1_clear(int_flg));
	if (spi2c->transfer_uses_dma) {
		dma_flg = sp_i2cm_get_dma_int_flag(sr) & SP_I2C_DMA_INT_ALL_FLG;
		sp_i2cm_dma_int_flag_clear(sr, dma_flg);
	}

	/* The INT register has unrelated status bits always set: test only the handled flags. */
	if (!(int_flg & (SP_I2C_INT_DONE_FLG | SP_I2C_INT_ADDR_NACK_FLG |
			 SP_I2C_INT_DATA_NACK_FLG | SP_I2C_INT_SCL_HOLD_TOO_LONG_FLG)) &&
	    !(dma_flg & (SP_I2C_DMA_INT_DMA_DONE_FLG | SP_I2C_DMA_INT_ERR_FLG))) {
		return IRQ_NONE;
	}

	/* DONE may have come in an earlier interrupt of this transfer, before DMA_DONE. */
	i2c_done = spi2c->i2c_done || (int_flg & SP_I2C_INT_DONE_FLG);

	/* Only a NACK overrides DONE; FIFO-empty is no error as the hardware refills the FIFO. */
	if (int_flg & SP_I2C_INT_ADDR_NACK_FLG) {
		dev_dbg(spi2c->dev, "Addr NACK\n");
		error = -ENXIO;
	} else if (int_flg & SP_I2C_INT_DATA_NACK_FLG) {
		dev_dbg(spi2c->dev, "Data NACK\n");
		error = -EIO;
	} else if (!i2c_done && (int_flg & SP_I2C_INT_SCL_HOLD_TOO_LONG_FLG)) {
		error = -ETIMEDOUT;
	} else if (dma_flg & SP_I2C_DMA_INT_ERR_FLG) {
		/* Even with DMA_DONE: the data may be short. */
		error = -EIO;
	}

	if (error && !spi2c->transfer_error)
		spi2c->transfer_error = error;
	if (int_flg & SP_I2C_INT_DONE_FLG)
		spi2c->i2c_done = true;
	if (dma_flg & SP_I2C_DMA_INT_DMA_DONE_FLG)
		spi2c->dma_done = true;

	if (spi2c->transfer_error ||
	    (spi2c->i2c_done && (spi2c->dma_done || !spi2c->transfer_uses_dma)))
		complete(&spi2c->transfer_done);

	return IRQ_HANDLED;
}

/* Twice the bus time of the transfer, plus the adapter timeout for clock stretching. */
static unsigned long sp_i2c_timeout_jiffies(struct sp_i2c_dev *spi2c, unsigned int data_bytes)
{
	u64 bus_bits = (u64)(data_bytes + 2) * SP_I2C_BITS_PER_BYTE;
	unsigned int bus_usecs = DIV_ROUND_UP_ULL(bus_bits * USEC_PER_SEC, spi2c->bus_freq_hz);

	return 2 * usecs_to_jiffies(bus_usecs) + spi2c->adap.timeout;
}

static int sp_i2c_wait_transfer(struct sp_i2c_dev *spi2c, unsigned long timeout)
{
	void __iomem *sr = spi2c->i2c_regs;
	unsigned long remaining;

	remaining = wait_for_completion_timeout(&spi2c->transfer_done, timeout);

	/*
	 * Quiesce the handler so it cannot touch the next transfer's state:
	 * clear the status before synchronize_irq(), so a handler that is
	 * still pending sees no flags and returns IRQ_NONE. The read-back
	 * makes sure the posted writes have reached the controller.
	 */
	writel(0, sr + SP_I2C_INT_EN0_REG);
	if (spi2c->transfer_uses_dma)
		sp_i2cm_dma_int_en_set(sr, 0);
	sp_i2cm_status_clear(sr, SP_I2C_CTL1_ALL_CLR);
	if (spi2c->transfer_uses_dma)
		sp_i2cm_dma_int_flag_clear(sr, SP_I2C_DMA_INT_ALL_FLG);
	readl(sr + SP_I2C_INT_EN0_REG);
	synchronize_irq(spi2c->irq);

	if (!remaining) {
		dev_err(spi2c->dev, "transfer timeout\n");
		return -ETIMEDOUT;
	}
	return spi2c->transfer_error;
}

static void sp_i2c_start_common(struct sp_i2c_dev *spi2c, unsigned int addr, bool uses_dma)
{
	void __iomem *sr = spi2c->i2c_regs;

	sp_i2cm_reset(sr);
	sp_i2cm_status_clear(sr, SP_I2C_CTL1_ALL_CLR);
	if (uses_dma)
		sp_i2cm_dma_int_flag_clear(sr, SP_I2C_DMA_INT_ALL_FLG);

	reinit_completion(&spi2c->transfer_done);
	spi2c->transfer_uses_dma = uses_dma;
	spi2c->i2c_done = false;
	spi2c->dma_done = false;
	spi2c->transfer_error = 0;

	sp_i2cm_addr_freq_set(sr, spi2c->clock_divider, addr);
	sp_i2cm_scl_delay_set(sr, SP_I2C_SCL_DELAY);
}

static int sp_i2c_pio_xfer(struct sp_i2c_dev *spi2c, unsigned int addr,
			   const u8 *write_buf, u16 write_len, u8 *read_buf, u16 read_len)
{
	void __iomem *sr = spi2c->i2c_regs;
	int error;

	sp_i2c_start_common(spi2c, addr, false);
	sp_i2cm_trans_cnt_set(sr, write_len, read_len);
	sp_i2cm_active_mode_set(sr, 0);

	if (!read_len) {
		sp_i2cm_rw_mode_set(sr, I2C_WRITE_MODE);
		sp_i2c_fifo_write(sr, write_buf, write_len);
	} else if (write_len) {
		sp_i2c_fifo_write(sr, write_buf, write_len);
		sp_i2cm_rw_mode_set(sr, I2C_RESTART_MODE);
	} else {
		sp_i2cm_rw_mode_set(sr, I2C_READ_MODE);
	}

	writel(SP_I2C_EN0_TRANSFER_INTS, sr + SP_I2C_INT_EN0_REG);
	sp_i2cm_manual_trigger(sr);

	error = sp_i2c_wait_transfer(spi2c, sp_i2c_timeout_jiffies(spi2c, write_len + read_len));
	if (!error && read_len)
		sp_i2c_fifo_read(sr, read_buf, read_len);
	sp_i2cm_reset(sr);
	return error;
}

static int sp_i2c_dma_xfer(struct sp_i2c_dev *spi2c, unsigned int addr,
			   const u8 *write_buf, u16 write_len, struct i2c_msg *msg)
{
	void __iomem *sr = spi2c->i2c_regs;
	bool is_read = msg->flags & I2C_M_RD;
	enum dma_data_direction direction = is_read ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
	dma_addr_t dma_addr;
	u8 *buf;
	int error;

	/*
	 * Always use a driver-owned buffer, even for I2C_M_DMA_SAFE messages:
	 * if a DMA engine gets wedged, we only find out after the transfer,
	 * and a stuck engine may still use the caller's buffer after it is freed.
	 */
	buf = kzalloc(msg->len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	if (!is_read)
		memcpy(buf, msg->buf, msg->len);

	dma_addr = dma_map_single(spi2c->dev, buf, msg->len, direction);
	if (dma_mapping_error(spi2c->dev, dma_addr)) {
		kfree(buf);
		return -ENOMEM;
	}

	sp_i2c_start_common(spi2c, addr, true);

	if (is_read) {
		if (write_len) {
			/* Data-sheet 31.7.6: DMA mode, no manual mode, then a manual trigger. */
			sp_i2cm_active_mode_set(sr, SP_I2C_MODE_DMA_MODE);
			sp_i2cm_rw_mode_set(sr, I2C_RESTART_MODE);
			sp_i2cm_trans_cnt_set(sr, write_len, msg->len);
			sp_i2c_fifo_write(sr, write_buf, write_len);
		} else {
			/* Data-sheet 31.7.4: the length is DMA_LENGTH; CTL7 is not used here. */
			sp_i2cm_active_mode_set(sr, SP_I2C_MODE_MANUAL_MODE | SP_I2C_MODE_DMA_MODE);
			sp_i2cm_rw_mode_set(sr, I2C_READ_MODE);
		}
		writel(SP_I2C_EN0_TRANSFER_INTS, sr + SP_I2C_INT_EN0_REG);
		sp_i2cm_dma_addr_set(sr, dma_addr);
		sp_i2cm_dma_len_set(sr, msg->len);
		sp_i2cm_dma_rw_mode_set(sr, I2C_DMA_WRITE_MODE);
		sp_i2cm_dma_int_en_set(sr, SP_I2C_DMA_EN_DMA_DONE_INT | SP_I2C_DMA_EN_ERR_INTS);
		sp_i2cm_dma_go_set(sr);
		if (write_len)
			sp_i2cm_manual_trigger(sr);
	} else {
		/* Data-sheet 31.7.5: the length is DMA_LENGTH; CTL7 is not used here. */
		sp_i2cm_active_mode_set(sr, SP_I2C_MODE_MANUAL_MODE | SP_I2C_MODE_DMA_MODE);
		sp_i2cm_rw_mode_set(sr, I2C_WRITE_MODE);
		writel(SP_I2C_EN0_TRANSFER_INTS, sr + SP_I2C_INT_EN0_REG);
		sp_i2cm_dma_addr_set(sr, dma_addr);
		sp_i2cm_dma_len_set(sr, msg->len);
		/* The DMA mode is named for the DMA engine: an I2C write is a DMA read. */
		sp_i2cm_dma_rw_mode_set(sr, I2C_DMA_READ_MODE);
		sp_i2cm_dma_int_en_set(sr, SP_I2C_DMA_EN_DMA_DONE_INT | SP_I2C_DMA_EN_ERR_INTS);
		sp_i2cm_dma_go_set(sr);
	}

	error = sp_i2c_wait_transfer(spi2c, sp_i2c_timeout_jiffies(spi2c, write_len + msg->len));
	if (error && sp_i2cm_dma_soft_reset(sr)) {
		spi2c->dma_wedged = true;
		/* Block module unload; with suppress_bind_attrs there is no sysfs unbind either. */
		__module_get(THIS_MODULE);
		dev_err(spi2c->dev, "DMA is stuck, the controller needs a hardware reset\n");
		/*
		 * Keep the buffer and its mapping for good: the engine may still use
		 * them, and only a hardware reset (a reboot, not kexec) makes freeing safe.
		 */
		kmemleak_not_leak(buf);
		return error;
	}

	sp_i2cm_reset(sr);
	dma_unmap_single(spi2c->dev, dma_addr, msg->len, direction);
	if (is_read && !error)
		memcpy(msg->buf, buf, msg->len);
	kfree(buf);
	return error;
}

static int sp_master_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct sp_i2c_dev *spi2c = adap->algo_data;
	struct i2c_msg *data_msg = &msgs[num - 1];
	const u8 *write_buf = NULL;
	u16 write_len = 0;
	int error, i;

	/* PIO too: a stuck DMA engine may still feed or drain the shared FIFO. */
	if (spi2c->dma_wedged)
		return -EIO;

	for (i = 0; i < num; i++) {
		if (msgs[i].flags & (I2C_M_TEN | I2C_M_NOSTART | I2C_M_RECV_LEN))
			return -EOPNOTSUPP;
	}

	if (num == 2) {
		write_buf = msgs[0].buf;
		write_len = msgs[0].len;
	}

	if (data_msg->len >= SP_I2C_DMA_THRESHOLD_BYTES)
		error = sp_i2c_dma_xfer(spi2c, data_msg->addr, write_buf, write_len, data_msg);
	else if (data_msg->flags & I2C_M_RD)
		error = sp_i2c_pio_xfer(spi2c, data_msg->addr, write_buf, write_len,
					data_msg->buf, data_msg->len);
	else
		error = sp_i2c_pio_xfer(spi2c, data_msg->addr, data_msg->buf, data_msg->len,
					NULL, 0);

	return error ?: num;
}

static u32 sp_functionality(struct i2c_adapter *adap)
{
	/* The controller can't send an address-only (zero-length) message. */
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_adapter_quirks sp_i2c_quirks = {
	.flags = I2C_AQ_COMB_WRITE_THEN_READ | I2C_AQ_NO_ZERO_LEN,
	.max_comb_1st_msg_len = SP_I2C_FIFO_BYTES,
};

static const struct i2c_algorithm sp_algorithm = {
	.xfer		= sp_master_xfer,
	.functionality	= sp_functionality,
};

static const struct of_device_id sp_i2c_of_match[] = {
	{ .compatible = "sunplus,sp7021-i2cm" },
	{ }
};
MODULE_DEVICE_TABLE(of, sp_i2c_of_match);

static int sp_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sp_i2c_dev *spi2c;
	struct reset_control *rstc;
	struct i2c_adapter *adap;
	struct clk *clk;
	u32 clock_frequency = I2C_MAX_STANDARD_MODE_FREQ;
	int ret;

	spi2c = devm_kzalloc(dev, sizeof(*spi2c), GFP_KERNEL);
	if (!spi2c)
		return -ENOMEM;

	spi2c->dev = dev;
	of_property_read_u32(dev->of_node, "clock-frequency", &clock_frequency);
	if (clock_frequency < SP_I2C_MIN_CLOCK_FREQ || clock_frequency > I2C_MAX_FAST_MODE_FREQ)
		return dev_err_probe(dev, -EINVAL, "clock-frequency %u Hz out of range\n",
				     clock_frequency);
	spi2c->bus_freq_hz = clock_frequency;
	spi2c->clock_divider = DIV_ROUND_UP(SP_I2C_SOURCE_CLK_HZ, clock_frequency) - 1;

	spi2c->i2c_regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spi2c->i2c_regs))
		return PTR_ERR(spi2c->i2c_regs);

	spi2c->irq = platform_get_irq(pdev, 0);
	if (spi2c->irq < 0)
		return spi2c->irq;

	clk = devm_clk_get_enabled(dev, "core");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to get and enable clock\n");

	/* The DMA engine also needs the clock and reset of controller 0. */
	clk = devm_clk_get_enabled(dev, "dma");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to get and enable DMA clock\n");

	/* Shared: the reset of controller 0 must stay released while any controller is bound. */
	rstc = devm_reset_control_get_shared_deasserted(dev, "core");
	if (IS_ERR(rstc))
		return dev_err_probe(dev, PTR_ERR(rstc), "failed to get and deassert reset\n");

	rstc = devm_reset_control_get_shared_deasserted(dev, "dma");
	if (IS_ERR(rstc))
		return dev_err_probe(dev, PTR_ERR(rstc), "failed to get and deassert DMA reset\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	init_completion(&spi2c->transfer_done);

	/* Stop and quiet what the bootloader or a previous kernel left, before the IRQ can fire. */
	writel(0, spi2c->i2c_regs + SP_I2C_INT_EN0_REG);
	writel(0, spi2c->i2c_regs + SP_I2C_INT_EN1_REG);
	writel(0, spi2c->i2c_regs + SP_I2C_INT_EN2_REG);
	sp_i2cm_dma_int_en_set(spi2c->i2c_regs, 0);
	ret = sp_i2cm_dma_soft_reset(spi2c->i2c_regs);
	if (ret)
		return dev_err_probe(dev, ret,
				     "DMA engine failed to reset; a hardware reset is needed\n");
	sp_i2cm_status_clear(spi2c->i2c_regs, SP_I2C_CTL1_ALL_CLR);
	sp_i2cm_dma_int_flag_clear(spi2c->i2c_regs, SP_I2C_DMA_INT_ALL_FLG);

	ret = devm_request_irq(dev, spi2c->irq, _sp_i2cm_irqevent_handler,
			       0, dev_name(dev), spi2c);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	/* The core derives the adapter number from the "i2c" DT alias. */
	adap = &spi2c->adap;
	strscpy(adap->name, "Sunplus SP7021 I2C", sizeof(adap->name));
	adap->algo = &sp_algorithm;
	adap->quirks = &sp_i2c_quirks;
	adap->owner = THIS_MODULE;
	adap->algo_data = spi2c;
	adap->dev.parent = dev;
	adap->dev.of_node = dev->of_node;

	sp_i2cm_reset(spi2c->i2c_regs);

	ret = devm_i2c_add_adapter(dev, adap);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add I2C controller\n");

	return 0;
}

static struct platform_driver sp_i2c_driver = {
	.probe	= sp_i2c_probe,
	.driver	= {
		.name		= "sunplus-i2cm",
		.of_match_table	= sp_i2c_of_match,
		/* Keep a wedged controller bound: no sysfs unbind (see sp_i2c_dma_xfer()). */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(sp_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Li-hao Kuo <lhjeff911@gmail.com>");
MODULE_DESCRIPTION("Sunplus SP7021 I2C master driver");
