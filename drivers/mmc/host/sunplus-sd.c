// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sunplus SP7021 SD/SDIO host controller driver
 * Copyright (C) Sunplus Technology Co., Ltd.
 * Author: Li-hao Kuo <lhjeff911@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/sizes.h>
#include <linux/workqueue.h>

#define SPSDC_MIN_CLK			400000
#define SPSDC_MAX_CLK			52000000
#define SPSDC_MAX_BLK_COUNT		65536
#define SPSDC_INT_MAX_CLK		25000000
#define SPSDC_PAGE_SIZE			512
#define SPSDC_DMA_INT_THRESHOLD		1024
#define SPSDC_SDIO_1BIT_MAX_BYTES	16
#define SPSDC_SDIO_WIDTH_UNKNOWN	(-1)
#define SPSDC_SDIO_BOUNCE_SIZE		SZ_64K
#define SPSDC_SDIO_MAX_SEGS		128
#define SPSDC_AUTOSUSPEND_MS		50
#define SPSDC_TIMEOUT_US		(5 * USEC_PER_SEC)
#define SPSDC_SPIN_US			200	/* commands normally finish within this */
#define SPSDC_POLL_SLEEP_US		100
#define SPSDC_DMA_RST_TIMEOUT_US	(10 * USEC_PER_MSEC)

#define SPSD2_MEDIA_TYPE_REG		0x0000
#define SPSDC_MEDIA_MASK		GENMASK(2, 0)
#define SPSDC_MEDIA_SD			6

#define SPSD2_SDRAM_SECTOR_SIZE_REG	0x0010
#define SPSDC_MAX_DMA_MEMORY_SECTORS	8

#define SPSD2_SDRAM_SECTOR_ADDR_REG	0x001C
#define SPSD2_SD_INT_REG		0x00B0
#define SPSDC_SDINT_SDCMPEN		BIT(0)
#define SPSDC_SDINT_SDCMP		BIT(1)
#define SPSDC_SDINT_SDCMP_CLR		BIT(2)
#define SPSDC_SDINT_SDIOEN		BIT(4)
#define SPSDC_SDINT_SDIO		BIT(5)
#define SPSDC_SDINT_SDIO_CLR		BIT(6)

#define SPSD2_SD_PAGE_NUM_REG		0x00B4
#define SPSD2_SD_CONF0_REG		0x00B8
#define SPSDC_CONF0_SDPIO_MODE		BIT(0)
#define SPSDC_CONF0_SDLEN_MODE		BIT(2)
#define SPSDC_CONF0_TRANS_MODE		GENMASK(5, 4)
#define SPSDC_TRANS_WR_MODE		1
#define SPSDC_TRANS_RD_MODE		2
#define SPSDC_CONF0_AUTORSP		BIT(6)
#define SPSDC_CONF0_CMDDUMMY		BIT(7)
#define SPSDC_CONF0_RSPCHK		BIT(8)

#define SPSD2_SDIO_CTRL_REG		0x00BC
#define SPSDC_SDIO_CTRL_MULTI_TRIG	BIT(6)

#define SPSD2_SD_RST_REG		0x00C0
#define SPSDC_RST_SDIO			BIT(2)
#define SPSDC_RST_CRC			BIT(1)
#define SPSDC_RST_SD			BIT(0)
#define SPSDC_RST_ALL			(SPSDC_RST_SDIO | SPSDC_RST_CRC | SPSDC_RST_SD)

#define SPSD2_SD_CONF_REG		0x00C4
#define SPSDC_CONF_CLK_DIV		GENMASK(11, 0)
#define SPSDC_CONF_4BIT_MODE		BIT(12)
#define SPSDC_CONF_SDRSP_TYPE		BIT(13)
#define SPSDC_CONF_SDMMC_MODE		BIT(16)
#define SPSDC_CONF_MMC8BIT		BIT(18)
#define SPSDC_CONF_SDIO_MODE		BIT(20)

#define SPSD2_SD_CTRL_REG		0x00C8
#define SPSDC_CTRL_CMD_TRIG		BIT(0)

#define SPSD2_SD_STATUS_REG		0x00CC
#define SPSDC_STS_RSP_BUF_FULL		BIT(1)
#define SPSDC_STS_RSP_TIMEOUT		BIT(6)
#define SPSDC_STS_CARD_CRC_TIMEOUT	BIT(7)
#define SPSDC_STS_STB_TIMEOUT		BIT(8)
#define SPSDC_STS_RSP_CRC7_ERR		BIT(9)
#define SPSDC_STS_CRC_TOKEN_ERR		BIT(10)
#define SPSDC_STS_RDATA_CRC16_ERR	BIT(11)

#define SPSD2_SD_STATE_REG		0x00D0
#define SPSDC_STATE_ERROR		BIT(13)
#define SPSDC_STATE_FINISH		BIT(14)

#define SPSD2_BLOCKSIZE_REG		0x00D4
#define SPSD2_SD_TIMING_CONF0_REG	0x00DC
#define SPSDC_TIMING_CONF0_HS_EN	BIT(11)
#define SPSDC_TIMING_CONF0_WRTD		GENMASK(14, 12)

#define SPSD2_SD_CMD_BUF0_REG		0x00EC
#define SPSD2_SD_CMD_BUF1_REG		0x00F0
#define SPSD2_SD_CMD_BUF2_REG		0x00F4
#define SPSD2_SD_CMD_BUF3_REG		0x00F8
#define SPSD2_SD_CMD_BUF4_REG		0x00FC
#define SPSD2_SD_RSP_BUF0_3_REG	0x0100
#define SPSD2_SD_RSP_BUF4_5_REG	0x0104
#define SPSDC_RSP_BUF4_5_MASK		GENMASK(15, 0)
#define SPSD2_DMA_SRCDST_REG		0x0204
#define SPSDC_DMA_DST			GENMASK(6, 4)
#define SPSDC_DMA_SRC			GENMASK(2, 0)
#define SPSDC_DMA_SDRAM			1
#define SPSDC_DMA_CARD			2
#define SPSD2_DMA_SIZE_REG		0x0208
#define SPSD2_DMA_STOP_RST_REG		0x020c
#define SPSDC_DMA_RST_CHAN		BIT(2)
#define SPSDC_HW_DMA_RST		BIT(1)
#define SPSD2_DMA_CTRL_REG		0x0210
#define SPSDC_DMA_CTRL_IDLE		BIT(0)
#define SPSD2_DMA_BASE_ADDR0_REG	0x0214
#define SPSD2_DMA_BASE_ADDR16_REG	0x0218

#define spsdc_wait_rspbuf_full(host) spsdc_wait_sdstatus(host, SPSDC_STS_RSP_BUF_FULL)

enum spsdc_mode {
	SPSDC_MODE_SD = 0,
	SPSDC_MODE_SDIO = 1,
};

struct spsdc_host {
	void __iomem *base;
	struct clk *clk;
	struct reset_control *rstc;
	bool clk_enabled;
	bool broken;		/* the DMA could not be stopped */
	struct mmc_host *mmc;
	struct mmc_request *mrq;
	int irq;
	int mode;
	int use_int;
	int power_state;	/* the power mode that is fully set up */
	bool pm_ref;		/* set_ios holds a runtime PM reference */
	int dma_use_int;
	int sdio_width;		/* card and controller width, or SPSDC_SDIO_WIDTH_UNKNOWN */
	spinlock_t lock;	/* serialises SD_INT_REG updates */
	bool sdio_irq_enabled;
	void *bounce_buf;	/* SDIO mode only */
	dma_addr_t bounce_dma;
	bool bounced;		/* the current request uses bounce_buf */
	struct delayed_work timeout_work;	/* completes a request whose IRQ never came */
	/* Under lock: which request each completion source belongs to. */
	u32 request_id;		/* of host->mrq */
	u32 irq_request_id;
	u32 timeout_request_id;
};

static void spsdc_update_int(struct spsdc_host *host, u32 clear, u32 set)
{
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&host->lock, flags);
	value = readl(host->base + SPSD2_SD_INT_REG);
	value &= ~clear;
	value |= set;
	writel(value, host->base + SPSD2_SD_INT_REG);
	spin_unlock_irqrestore(&host->lock, flags);
}

/* Call with host->lock held. */
static void __spsdc_enable_sdio_irq(struct spsdc_host *host, bool enable)
{
	u32 value = readl(host->base + SPSD2_SD_INT_REG);

	host->sdio_irq_enabled = enable;
	/* Drop a latch from while masked: INT MULTI TRIG latches again while DAT1 is low. */
	value |= SPSDC_SDINT_SDIO_CLR;
	if (enable)
		value |= SPSDC_SDINT_SDIOEN;
	else
		value &= ~SPSDC_SDINT_SDIOEN;
	writel(value, host->base + SPSD2_SD_INT_REG);
}

/* Re-apply the SDIO interrupt enable after a block reset. */
static void spsdc_restore_sdio_irq(struct spsdc_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	__spsdc_enable_sdio_irq(host, host->sdio_irq_enabled);
	spin_unlock_irqrestore(&host->lock, flags);
}

static int spsdc_wait_finish(struct spsdc_host *host)
{
	u32 state;
	int ret;

	ret = readl_poll_timeout_atomic(host->base + SPSD2_SD_STATE_REG, state,
					state & (SPSDC_STATE_FINISH | SPSDC_STATE_ERROR),
					0, SPSDC_SPIN_US);
	if (ret)
		ret = readl_poll_timeout(host->base + SPSD2_SD_STATE_REG, state,
					 state & (SPSDC_STATE_FINISH | SPSDC_STATE_ERROR),
					 SPSDC_POLL_SLEEP_US, SPSDC_TIMEOUT_US);
	if (ret)
		return ret;
	return (state & SPSDC_STATE_FINISH) ? 0 : -EIO;
}

static int spsdc_check_sdstatus(struct spsdc_host *host, u32 status_bit)
{
	if (readl(host->base + SPSD2_SD_STATUS_REG) & status_bit)
		return 1;
	if (readl(host->base + SPSD2_SD_STATE_REG) & SPSDC_STATE_ERROR)
		return -EIO;
	return 0;
}

static int spsdc_wait_sdstatus(struct spsdc_host *host, u32 status_bit)
{
	int val, ret;

	ret = read_poll_timeout_atomic(spsdc_check_sdstatus, val, val, 0,
				       SPSDC_SPIN_US, false, host, status_bit);
	if (ret)
		ret = read_poll_timeout(spsdc_check_sdstatus, val, val,
					SPSDC_POLL_SLEEP_US, SPSDC_TIMEOUT_US, false,
					host, status_bit);
	if (ret)
		return ret;
	return val < 0 ? val : 0;
}

static int spsdc_get_rsp(struct spsdc_host *host, struct mmc_command *cmd)
{
	u32 value0_3, value4_5;

	if (unlikely(!(cmd->flags & MMC_RSP_PRESENT)))
		return 0;
	if (unlikely(cmd->flags & MMC_RSP_136)) {
		if (spsdc_wait_rspbuf_full(host))
			return -ETIMEDOUT;
		value0_3 = readl(host->base + SPSD2_SD_RSP_BUF0_3_REG);
		value4_5 = readl(host->base + SPSD2_SD_RSP_BUF4_5_REG) & SPSDC_RSP_BUF4_5_MASK;
		cmd->resp[0] = (value0_3 << 8) | (value4_5 >> 8);
		cmd->resp[1] = value4_5 << 24;
		if (spsdc_wait_rspbuf_full(host))
			return -ETIMEDOUT;
		value0_3 = readl(host->base + SPSD2_SD_RSP_BUF0_3_REG);
		value4_5 = readl(host->base + SPSD2_SD_RSP_BUF4_5_REG) & SPSDC_RSP_BUF4_5_MASK;
		cmd->resp[1] |= value0_3 >> 8;
		cmd->resp[2] = value0_3 << 24;
		cmd->resp[2] |= value4_5 << 8;
		if (spsdc_wait_rspbuf_full(host))
			return -ETIMEDOUT;
		value0_3 = readl(host->base + SPSD2_SD_RSP_BUF0_3_REG);
		value4_5 = readl(host->base + SPSD2_SD_RSP_BUF4_5_REG) & SPSDC_RSP_BUF4_5_MASK;
		cmd->resp[2] |= value0_3 >> 24;
		cmd->resp[3] = value0_3 << 8;
		cmd->resp[3] |= value4_5 >> 8;
	} else {
		if (spsdc_wait_rspbuf_full(host))
			return -ETIMEDOUT;
		value0_3 = readl(host->base + SPSD2_SD_RSP_BUF0_3_REG);
		value4_5 = readl(host->base + SPSD2_SD_RSP_BUF4_5_REG) & SPSDC_RSP_BUF4_5_MASK;
		cmd->resp[0] = (value0_3 << 8) | (value4_5 >> 8);
		cmd->resp[1] = value4_5 << 24;
	}
	return 0;
}

static void spsdc_set_bus_clk(struct spsdc_host *host, int clk)
{
	unsigned int clkdiv;
	int f_min = host->mmc->f_min;
	int f_max = host->mmc->f_max;
	u32 value = readl(host->base + SPSD2_SD_CONF_REG);

	/*
	 * The controller can't stop SDCLK. A clock-off request (0) at power-up
	 * runs at f_min, and after power-off SDCLK keeps its last rate until
	 * runtime suspend gates the controller clock.
	 */
	if (clk < f_min)
		clk = f_min;
	if (clk > f_max)
		clk = f_max;

	clkdiv = DIV_ROUND_UP(clk_get_rate(host->clk), clk) - 1;
	if (clkdiv > FIELD_MAX(SPSDC_CONF_CLK_DIV))
		clkdiv = FIELD_MAX(SPSDC_CONF_CLK_DIV);
	host->mmc->actual_clock = clk_get_rate(host->clk) / (clkdiv + 1);

	value &= ~SPSDC_CONF_CLK_DIV;
	value |= FIELD_PREP(SPSDC_CONF_CLK_DIV, clkdiv);
	writel(value, host->base + SPSD2_SD_CONF_REG);
	host->use_int = host->mmc->actual_clock <= SPSDC_INT_MAX_CLK;
}

static void spsdc_set_bus_timing(struct spsdc_host *host, unsigned int timing)
{
	u32 value = readl(host->base + SPSD2_SD_TIMING_CONF0_REG);
	int clkdiv = FIELD_GET(SPSDC_CONF_CLK_DIV, readl(host->base + SPSD2_SD_CONF_REG));
	int delay = min_t(int, clkdiv / 2, FIELD_MAX(SPSDC_TIMING_CONF0_WRTD));

	value &= ~(SPSDC_TIMING_CONF0_HS_EN | SPSDC_TIMING_CONF0_WRTD);
	switch (timing) {
	case MMC_TIMING_SD_HS:
	case MMC_TIMING_MMC_HS:
		value |= SPSDC_TIMING_CONF0_HS_EN |
			FIELD_PREP(SPSDC_TIMING_CONF0_WRTD, delay);
		break;
	}
	writel(value, host->base + SPSD2_SD_TIMING_CONF0_REG);
}

static void spsdc_set_bus_width(struct spsdc_host *host, int width)
{
	u32 value = readl(host->base + SPSD2_SD_CONF_REG);

	switch (width) {
	case MMC_BUS_WIDTH_8:
		value &= ~SPSDC_CONF_4BIT_MODE;
		value |= SPSDC_CONF_MMC8BIT;
		break;
	case MMC_BUS_WIDTH_4:
		value |= SPSDC_CONF_4BIT_MODE;
		value &= ~SPSDC_CONF_MMC8BIT;
		break;
	default:
		value &= ~(SPSDC_CONF_4BIT_MODE | SPSDC_CONF_MMC8BIT);
		break;
	}
	writel(value, host->base + SPSD2_SD_CONF_REG);
}

static void spsdc_select_mode(struct spsdc_host *host)
{
	u32 value = readl(host->base + SPSD2_SD_CONF_REG);

	/* Without sdmmcmode, legacy-timing data is sampled on the falling clock edge. */
	value |= SPSDC_CONF_SDMMC_MODE;
	switch (host->mode) {
	case SPSDC_MODE_SDIO:
		value |= SPSDC_CONF_SDIO_MODE;
		writel(value, host->base + SPSD2_SD_CONF_REG);
		value = readl(host->base + SPSD2_SDIO_CTRL_REG);
		value |= SPSDC_SDIO_CTRL_MULTI_TRIG;
		writel(value, host->base + SPSD2_SDIO_CTRL_REG);
		break;
	case SPSDC_MODE_SD:
	default:
		value &= ~SPSDC_CONF_SDIO_MODE;
		host->mode = SPSDC_MODE_SD;
		writel(value, host->base + SPSD2_SD_CONF_REG);
		break;
	}
}

static int spsdc_controller_init(struct spsdc_host *host)
{
	u32 value;
	int ret;

	ret = reset_control_assert(host->rstc);
	if (!ret) {
		usleep_range(1000, 1250);
		ret = reset_control_deassert(host->rstc);
	}
	if (ret) {
		dev_err(mmc_dev(host->mmc), "controller reset failed: %d\n", ret);
		return ret;
	}
	value = readl(host->base + SPSD2_MEDIA_TYPE_REG);
	value &= ~SPSDC_MEDIA_MASK;
	value |= FIELD_PREP(SPSDC_MEDIA_MASK, SPSDC_MEDIA_SD);
	writel(value, host->base + SPSD2_MEDIA_TYPE_REG);
	spsdc_restore_sdio_irq(host);
	return 0;
}

static void spsdc_sw_reset(struct spsdc_host *host)
{
	unsigned long flags;
	u32 val;

	/* This clears pending SDIO interrupts, but SDIOEN survives. */
	writel(SPSDC_RST_ALL, host->base + SPSD2_SD_RST_REG);
	writel(SPSDC_DMA_RST_CHAN | SPSDC_HW_DMA_RST,
	       host->base + SPSD2_DMA_STOP_RST_REG);
	if (readl_poll_timeout(host->base + SPSD2_DMA_STOP_RST_REG, val,
			       !(val & SPSDC_DMA_RST_CHAN), 0,
			       SPSDC_DMA_RST_TIMEOUT_US)) {
		struct mmc_ios *ios = &host->mmc->ios;

		dev_err_ratelimited(mmc_dev(host->mmc),
				    "DMA channel reset timed out, resetting the controller\n");
		/* Only the block reset surely stops the DMA before its buffers are released. */
		if (spsdc_controller_init(host)) {
			/*
			 * Gating the clock is the last way left to stop the DMA: its logic
			 * runs on the card clock (data sheet, section 125.6).
			 */
			disable_irq_nosync(host->irq);
			/* Under the lock, a running spsdc_irq() finishes before the gate. */
			spin_lock_irqsave(&host->lock, flags);
			host->broken = true;
			clk_disable(host->clk);
			host->clk_enabled = false;
			spin_unlock_irqrestore(&host->lock, flags);
			dev_err(mmc_dev(host->mmc),
				"controller reset failed; clock gated and controller disabled\n");
			return;
		}
		spsdc_set_bus_clk(host, ios->clock);
		spsdc_set_bus_timing(host, ios->timing);
		spsdc_set_bus_width(host, ios->bus_width);
		spsdc_select_mode(host);
		/* The next request sets the card's SDIO width again. */
		host->sdio_width = SPSDC_SDIO_WIDTH_UNKNOWN;
		return;
	}
	writel(0, host->base + SPSD2_DMA_CTRL_REG);
	writel(SPSDC_DMA_CTRL_IDLE, host->base + SPSD2_DMA_CTRL_REG);
	writel(0, host->base + SPSD2_DMA_CTRL_REG);
}

static void spsdc_prepare_cmd(struct spsdc_host *host, struct mmc_command *cmd)
{
	u32 value;

	writeb(cmd->opcode | 0x40, host->base + SPSD2_SD_CMD_BUF0_REG);
	writeb((cmd->arg >> 24) & 0xff, host->base + SPSD2_SD_CMD_BUF1_REG);
	writeb((cmd->arg >> 16) & 0xff, host->base + SPSD2_SD_CMD_BUF2_REG);
	writeb((cmd->arg >>  8) & 0xff, host->base + SPSD2_SD_CMD_BUF3_REG);
	writeb((cmd->arg >>  0) & 0xff, host->base + SPSD2_SD_CMD_BUF4_REG);

	if (likely(!host->use_int || cmd->flags & MMC_RSP_136))
		spsdc_update_int(host, SPSDC_SDINT_SDCMPEN, SPSDC_SDINT_SDCMP_CLR);
	else
		spsdc_update_int(host, 0, SPSDC_SDINT_SDCMP_CLR | SPSDC_SDINT_SDCMPEN);

	value = readl(host->base + SPSD2_SD_CONF0_REG);
	value &= ~SPSDC_CONF0_TRANS_MODE;
	value |= SPSDC_CONF0_CMDDUMMY;
	if (likely(cmd->flags & MMC_RSP_PRESENT)) {
		value |= SPSDC_CONF0_AUTORSP;
	} else {
		value &= ~SPSDC_CONF0_AUTORSP;
		writel(value, host->base + SPSD2_SD_CONF0_REG);
		return;
	}
	if (likely(cmd->flags & MMC_RSP_CRC && !(cmd->flags & MMC_RSP_136)))
		value |= SPSDC_CONF0_RSPCHK;
	else
		value &= ~SPSDC_CONF0_RSPCHK;
	writel(value, host->base + SPSD2_SD_CONF0_REG);

	value = readl(host->base + SPSD2_SD_CONF_REG);
	if (unlikely(cmd->flags & MMC_RSP_136))
		value |= SPSDC_CONF_SDRSP_TYPE;
	else
		value &= ~SPSDC_CONF_SDRSP_TYPE;
	writel(value, host->base + SPSD2_SD_CONF_REG);
}

/*
 * Function drivers pass lists of arbitrary-length segments
 * (e.g. brcmfmac's frame aggregation), but the DMA engine only takes
 * whole blocks. Also, multi-segment DMA with SDIO block sizes other than
 * 512 bytes is untested. So in SDIO mode, transfer everything through
 * the bounce buffer, except for single whole-block segment transfers.
 */
static bool spsdc_needs_bounce(struct mmc_data *data)
{
	struct scatterlist *sg;
	int i;

	if (data->sg_len > 1)
		return true;
	for_each_sg(data->sg, sg, data->sg_len, i)
		if (sg->length % data->blksz)
			return true;
	return false;
}

/* Unmap the request's data and, for a bounced read, copy it to its buffers. */
static void spsdc_unmap_data(struct spsdc_host *host, struct mmc_data *data)
{
	int dma_direction = data->flags & MMC_DATA_READ ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
	unsigned int len = data->blocks * data->blksz;

	if (host->bounced) {
		dma_unmap_single(host->mmc->parent, host->bounce_dma, len, dma_direction);
		if ((data->flags & MMC_DATA_READ) && data->bytes_xfered)
			sg_copy_from_buffer(data->sg, data->sg_len, host->bounce_buf, len);
		host->bounced = false;
	} else {
		dma_unmap_sg(host->mmc->parent, data->sg, data->sg_len, dma_direction);
	}
}

static int spsdc_prepare_data(struct spsdc_host *host, struct mmc_data *data)
{
	int dma_direction = data->flags & MMC_DATA_READ ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
	unsigned int len = data->blocks * data->blksz;
	struct scatterlist *sg;
	dma_addr_t dma_addr;
	unsigned int dma_size;
	void __iomem *reg_addr;
	int i, count;
	u32 value;

	if (host->bounce_buf && spsdc_needs_bounce(data)) {
		if (unlikely(len > SPSDC_SDIO_BOUNCE_SIZE))
			return -EINVAL;
		if (!(data->flags & MMC_DATA_READ))
			sg_copy_to_buffer(data->sg, data->sg_len, host->bounce_buf, len);
		host->bounce_dma = dma_map_single(host->mmc->parent, host->bounce_buf,
						  len, dma_direction);
		if (dma_mapping_error(host->mmc->parent, host->bounce_dma))
			return -ENOMEM;
		host->bounced = true;
		count = 1;
	} else {
		count = dma_map_sg(host->mmc->parent, data->sg, data->sg_len,
				   dma_direction);
		if (unlikely(!count))
			return -ENOMEM;
		if (unlikely(count > SPSDC_MAX_DMA_MEMORY_SECTORS))
			goto unmap;
		/* Segment lengths are programmed in whole blocks. */
		for_each_sg(data->sg, sg, count, i)
			if (unlikely(sg_dma_len(sg) % data->blksz))
				goto unmap;
	}

	writel(data->blocks - 1, host->base + SPSD2_SD_PAGE_NUM_REG);
	writel(data->blksz - 1, host->base + SPSD2_BLOCKSIZE_REG);
	value = readl(host->base + SPSD2_SD_CONF0_REG);
	value &= ~SPSDC_CONF0_TRANS_MODE;
	if (data->flags & MMC_DATA_READ) {
		value &= ~(SPSDC_CONF0_AUTORSP | SPSDC_CONF0_CMDDUMMY);
		value |= FIELD_PREP(SPSDC_CONF0_TRANS_MODE, SPSDC_TRANS_RD_MODE);
		writel(FIELD_PREP(SPSDC_DMA_DST, SPSDC_DMA_SDRAM) |
		       FIELD_PREP(SPSDC_DMA_SRC, SPSDC_DMA_CARD),
		       host->base + SPSD2_DMA_SRCDST_REG);
	} else {
		value |= FIELD_PREP(SPSDC_CONF0_TRANS_MODE, SPSDC_TRANS_WR_MODE);
		writel(FIELD_PREP(SPSDC_DMA_DST, SPSDC_DMA_CARD) |
		       FIELD_PREP(SPSDC_DMA_SRC, SPSDC_DMA_SDRAM),
		       host->base + SPSD2_DMA_SRCDST_REG);
	}
	value |= SPSDC_CONF0_SDLEN_MODE;
	if (host->bounced) {
		writel(host->bounce_dma, host->base + SPSD2_DMA_BASE_ADDR0_REG);
		writel(host->bounce_dma >> 16, host->base + SPSD2_DMA_BASE_ADDR16_REG);
		writel(data->blocks - 1, host->base + SPSD2_SDRAM_SECTOR_SIZE_REG);
	}
	for_each_sg(data->sg, sg, host->bounced ? 0 : count, i) {
		dma_addr = sg_dma_address(sg);
		dma_size = sg_dma_len(sg) / data->blksz - 1;
		if (i == 0) {
			writel(dma_addr, host->base + SPSD2_DMA_BASE_ADDR0_REG);
			writel(dma_addr >> 16, host->base + SPSD2_DMA_BASE_ADDR16_REG);
			writel(dma_size, host->base + SPSD2_SDRAM_SECTOR_SIZE_REG);
		} else {
			reg_addr = host->base + SPSD2_SDRAM_SECTOR_ADDR_REG + (i - 1) * 8;
			writel(dma_addr, reg_addr);
			writel(dma_size, reg_addr + 4);
		}
	}
	value &= ~SPSDC_CONF0_SDPIO_MODE;
	writel(value, host->base + SPSD2_SD_CONF0_REG);
	writel(data->blksz - 1, host->base + SPSD2_DMA_SIZE_REG);
	if (!host->use_int && data->blksz * data->blocks > SPSDC_DMA_INT_THRESHOLD) {
		host->dma_use_int = 1;
		spsdc_update_int(host, 0, SPSDC_SDINT_SDCMPEN);
	}
	return 0;

unmap:
	dma_unmap_sg(host->mmc->parent, data->sg, data->sg_len, dma_direction);
	return -EINVAL;
}

static inline void spsdc_trigger_transaction(struct spsdc_host *host)
{
	u32 value = readl(host->base + SPSD2_SD_CTRL_REG);

	value |= SPSDC_CTRL_CMD_TRIG;
	writel(value, host->base + SPSD2_SD_CTRL_REG);
}

static int spsdc_send_stop_cmd(struct spsdc_host *host, struct mmc_command *stop)
{
	u32 value;
	int ret;

	spsdc_prepare_cmd(host, stop);
	spsdc_update_int(host, SPSDC_SDINT_SDCMPEN, 0);
	spsdc_trigger_transaction(host);
	ret = spsdc_wait_finish(host);
	if (ret) {
		value = readl(host->base + SPSD2_SD_STATUS_REG);
		stop->error = (value & SPSDC_STS_RSP_CRC7_ERR) ? -EILSEQ : -ETIMEDOUT;
		return ret;
	}
	ret = spsdc_get_rsp(host, stop);
	if (ret) {
		stop->error = ret;
		return ret;	/* the caller resets the controller */
	}
	return 0;
}

/* Read or write one byte of the card's CCCR with a CMD52, outside the core. */
static int spsdc_sdio_cccr_rw(struct spsdc_host *host, bool write,
			      unsigned int address, u8 value, u8 *result)
{
	struct mmc_command cmd = {};
	int ret;

	cmd.opcode = SD_IO_RW_DIRECT;
	cmd.arg = (write ? 0x80000000 : 0) | address << 9 | value;
	cmd.flags = MMC_RSP_R5;
	spsdc_prepare_cmd(host, &cmd);
	spsdc_update_int(host, SPSDC_SDINT_SDCMPEN, 0);
	spsdc_trigger_transaction(host);
	ret = spsdc_wait_finish(host);
	if (!ret)
		ret = spsdc_get_rsp(host, &cmd);
	if (ret) {
		spsdc_sw_reset(host);
		return ret;
	}
	if (cmd.resp[0] & (R5_ERROR | R5_FUNCTION_NUMBER | R5_OUT_OF_RANGE))
		return -EIO;
	*result = cmd.resp[0] & 0xff;
	return 0;
}

/*
 * Set the card's and the controller's bus width. On failure the two may
 * differ, so the width is marked unknown and the next request sets it again.
 */
static int spsdc_switch_sdio_bus_width(struct spsdc_host *host, int width)
{
	int ret;
	u8 ctrl;

	host->sdio_width = SPSDC_SDIO_WIDTH_UNKNOWN;
	ret = spsdc_sdio_cccr_rw(host, false, SDIO_CCCR_IF, 0, &ctrl);
	if (ret)
		return ret;

	ctrl &= ~SDIO_BUS_WIDTH_MASK;
	if (width == MMC_BUS_WIDTH_4)
		ctrl |= SDIO_BUS_WIDTH_4BIT;
	ret = spsdc_sdio_cccr_rw(host, true, SDIO_CCCR_IF, ctrl, &ctrl);
	if (ret)
		return ret;
	spsdc_set_bus_width(host, width);
	host->sdio_width = width;
	return 0;
}

/* Undo the 1-bit switch for a short CMD53; on failure the next request retries. */
static void spsdc_restore_sdio_bus_width(struct spsdc_host *host)
{
	if (!host->broken && host->sdio_width == MMC_BUS_WIDTH_1 &&
	    host->mmc->ios.bus_width == MMC_BUS_WIDTH_4)
		spsdc_switch_sdio_bus_width(host, MMC_BUS_WIDTH_4);
}

static int spsdc_check_error(struct spsdc_host *host, struct mmc_request *mrq,
			     int rsp_error)
{
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	int cmd_error = 0, data_error = 0;
	u32 state = readl(host->base + SPSD2_SD_STATE_REG);
	u32 status = 0;

	if (unlikely(!(state & (SPSDC_STATE_FINISH | SPSDC_STATE_ERROR)))) {
		/* spsdc_wait_finish() timed out; the controller may still be busy. */
		if (data)
			data_error = -ETIMEDOUT;
		else
			cmd_error = -ETIMEDOUT;
	} else if (unlikely(state & SPSDC_STATE_ERROR)) {
		status = readl(host->base + SPSD2_SD_STATUS_REG);
		/* The command phase failed: no response, or a bad one. */
		if (status & SPSDC_STS_RSP_TIMEOUT)
			cmd_error = -ETIMEDOUT;
		else if (status & SPSDC_STS_RSP_CRC7_ERR)
			cmd_error = -EILSEQ;
		/* The data phase failed; the core recovers these differently. */
		if (data) {
			if (status & (SPSDC_STS_STB_TIMEOUT | SPSDC_STS_CARD_CRC_TIMEOUT))
				data_error = -ETIMEDOUT;
			else if (status & (SPSDC_STS_CRC_TOKEN_ERR | SPSDC_STS_RDATA_CRC16_ERR))
				data_error = -EILSEQ;
		}
		if (!cmd_error && !data_error) {
			if (data)
				data_error = -EIO;
			else
				cmd_error = -EIO;
		}
	}
	/* No hardware error, but the response never arrived. */
	if (!cmd_error && !data_error && rsp_error)
		cmd_error = rsp_error;

	if (cmd_error || data_error) {
		cmd->error = cmd_error;
		if (data) {
			data->error = data_error;
			data->bytes_xfered = 0;
		}
		spsdc_sw_reset(host);
	} else if (data) {
		data->bytes_xfered = data->blocks * data->blksz;
	}
	return cmd_error ? cmd_error : data_error;
}

/*
 * Take the current request for completion if its ID matches. Only one
 * method succeeds for each request: the IRQ thread, the timeout, or
 * the polled path. Late callers get NULL.
 */
static struct mmc_request *spsdc_claim_request(struct spsdc_host *host, u32 request_id)
{
	struct mmc_request *mrq = NULL;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	if (host->mrq && host->request_id == request_id) {
		mrq = host->mrq;
		host->mrq = NULL;
	}
	spin_unlock_irqrestore(&host->lock, flags);
	return mrq;
}

static void spsdc_finish_request(struct spsdc_host *host, struct mmc_request *mrq)
{
	struct mmc_command *cmd;
	struct mmc_data *data;
	int rsp_error;
	u32 state;

	if (!mrq)
		return;

	cmd = mrq->cmd;
	data = mrq->data;
	state = readl(host->base + SPSD2_SD_STATE_REG);
	/* A controller that has not finished has no response to read. */
	rsp_error = state & (SPSDC_STATE_FINISH | SPSDC_STATE_ERROR) ?
		    spsdc_get_rsp(host, cmd) : 0;
	/* If there's an error, stop the controller before its DMA buffers are released. */
	spsdc_check_error(host, mrq, rsp_error);
	if (data) {
		spsdc_unmap_data(host, data);
		host->dma_use_int = 0;
	}
	/* A broken controller's clock is gated: send no more commands. */
	if (host->broken) {
		mmc_request_done(host->mmc, mrq);
		return;
	}
	if (mrq->stop) {
		if (spsdc_send_stop_cmd(host, mrq->stop))
			spsdc_sw_reset(host);
	}
	spsdc_restore_sdio_bus_width(host);
	mmc_request_done(host->mmc, mrq);
}

static irqreturn_t spsdc_irq(int irq, void *dev_id)
{
	struct spsdc_host *host = dev_id;
	irqreturn_t ret = IRQ_NONE;
	bool sdio = false;
	u32 value;

	spin_lock(&host->lock);
	/* Gated by spsdc_sw_reset(); the line is already masked. */
	if (!host->clk_enabled) {
		spin_unlock(&host->lock);
		return IRQ_NONE;
	}
	value = readl(host->base + SPSD2_SD_INT_REG);
	if ((value & SPSDC_SDINT_SDCMP) && (value & SPSDC_SDINT_SDCMPEN)) {
		writel(value | SPSDC_SDINT_SDCMP_CLR, host->base + SPSD2_SD_INT_REG);
		if (host->mrq) {
			host->irq_request_id = host->request_id;
			ret = IRQ_WAKE_THREAD;
		} else {
			ret = IRQ_HANDLED;	/* stale: its request has already been completed */
		}
	}
	if ((value & SPSDC_SDINT_SDIO) && (value & SPSDC_SDINT_SDIOEN)) {
		/* Masked until the core has run the function handlers. */
		__spsdc_enable_sdio_irq(host, false);
		sdio = true;
		if (ret == IRQ_NONE)
			ret = IRQ_HANDLED;
	}
	spin_unlock(&host->lock);

	if (sdio)
		sdio_signal_irq(host->mmc);
	return ret;
}

static irqreturn_t spsdc_irq_thread(int irq, void *dev_id)
{
	struct spsdc_host *host = dev_id;
	struct mmc_request *mrq;
	unsigned long flags;
	u32 request_id;

	spin_lock_irqsave(&host->lock, flags);
	request_id = host->irq_request_id;
	spin_unlock_irqrestore(&host->lock, flags);

	mrq = spsdc_claim_request(host, request_id);
	if (!mrq)
		return IRQ_HANDLED;
	/* A timeout run already in progress finds this request claimed and returns. */
	cancel_delayed_work_sync(&host->timeout_work);
	spsdc_finish_request(host, mrq);
	return IRQ_HANDLED;
}

static void spsdc_request_timeout(struct work_struct *work)
{
	struct spsdc_host *host = container_of(to_delayed_work(work),
					       struct spsdc_host, timeout_work);
	struct mmc_request *mrq;
	unsigned long flags;
	u32 request_id;

	spin_lock_irqsave(&host->lock, flags);
	request_id = host->timeout_request_id;
	spin_unlock_irqrestore(&host->lock, flags);

	mrq = spsdc_claim_request(host, request_id);
	if (!mrq)
		return;
	/* A late completion IRQ must not be taken for the next request. */
	spsdc_update_int(host, SPSDC_SDINT_SDCMPEN, SPSDC_SDINT_SDCMP_CLR);
	/* spsdc_check_error() reports a still-busy controller as -ETIMEDOUT. */
	spsdc_finish_request(host, mrq);
}

/* Complete a request that was never started on the hardware. */
static void spsdc_request_abort(struct spsdc_host *host, struct mmc_request *mrq)
{
	unsigned long flags;

	spsdc_restore_sdio_bus_width(host);
	spin_lock_irqsave(&host->lock, flags);
	host->mrq = NULL;
	spin_unlock_irqrestore(&host->lock, flags);
	mmc_request_done(host->mmc, mrq);
}

/* Fixed allowance for the card, plus twice the data's time on the bus. */
static unsigned long spsdc_calculate_timeout(struct spsdc_host *host,
					     struct mmc_data *data)
{
	u64 timeout_us = SPSDC_TIMEOUT_US;

	if (data) {
		u64 bits = (u64)data->blocks * data->blksz * 8;
		unsigned int width = 1 << host->mmc->ios.bus_width;

		timeout_us += div_u64(2 * bits * USEC_PER_SEC,
				      width * host->mmc->actual_clock);
	}
	return usecs_to_jiffies(timeout_us);
}

static void spsdc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct spsdc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd;
	struct mmc_data *data;
	int bus_width = mmc->ios.bus_width;
	bool completed_by_irq;
	unsigned long flags;
	u32 request_id;
	int ret;

	/* Unclocked after a failed resume, or broken with the DMA possibly still running. */
	if (!host->clk_enabled || host->broken) {
		mrq->cmd->error = -EIO;
		mmc_request_done(mmc, mrq);
		return;
	}
	/* An R2 response has no data phase, and the R2 path sets up no DMA. */
	if ((mrq->cmd->flags & MMC_RSP_136) && mrq->data) {
		mrq->cmd->error = -EINVAL;
		mmc_request_done(mmc, mrq);
		return;
	}
	/* CMD23 is not supported, and without it the transfer would not stop. */
	if (mrq->sbc) {
		mrq->sbc->error = -EINVAL;
		mmc_request_done(mmc, mrq);
		return;
	}
	spin_lock_irqsave(&host->lock, flags);
	host->mrq = mrq;
	request_id = ++host->request_id;
	spin_unlock_irqrestore(&host->lock, flags);
	data = mrq->data;
	cmd = mrq->cmd;
	/*
	 * On a 4-bit bus, the data block of a short read (2 clocks per byte
	 * plus 18) can end before the 48-clock R5 response. The controller
	 * then captures the response wrongly and reports a CRC error, so do
	 * short reads on a 1-bit bus. Writes are not affected: their data
	 * follows the response. The limit of 16 bytes keeps a margin over the
	 * 14 that this gives, as the card's response delay varies.
	 */
	if (host->mode == SPSDC_MODE_SDIO && bus_width == MMC_BUS_WIDTH_4 && data) {
		int width = MMC_BUS_WIDTH_4;

		if (cmd->opcode == SD_IO_RW_EXTENDED && (data->flags & MMC_DATA_READ) &&
		    data->blocks * data->blksz <= SPSDC_SDIO_1BIT_MAX_BYTES)
			width = MMC_BUS_WIDTH_1;
		if (host->sdio_width != width &&
		    spsdc_switch_sdio_bus_width(host, width)) {
			cmd->error = -EIO;
			spsdc_request_abort(host, mrq);
			return;
		}
	}

	spsdc_prepare_cmd(host, cmd);
	if (unlikely(cmd->flags & MMC_RSP_136)) {
		spsdc_trigger_transaction(host);
		ret = spsdc_get_rsp(host, cmd);
		spsdc_wait_finish(host);
		spsdc_check_error(host, mrq, ret);
		spsdc_claim_request(host, request_id);
		mmc_request_done(host->mmc, mrq);
	} else {
		if (data) {
			ret = spsdc_prepare_data(host, data);
			if (ret) {
				cmd->error = ret;
				data->error = ret;
				spsdc_request_abort(host, mrq);
				return;
			}
		}
		completed_by_irq = host->use_int || host->dma_use_int;
		if (completed_by_irq) {
			spin_lock_irqsave(&host->lock, flags);
			host->timeout_request_id = request_id;
			spin_unlock_irqrestore(&host->lock, flags);
			schedule_delayed_work(&host->timeout_work,
					      spsdc_calculate_timeout(host, data));
		}
		spsdc_trigger_transaction(host);
		if (!completed_by_irq) {
			spsdc_wait_finish(host);
			if (spsdc_claim_request(host, request_id))
				spsdc_finish_request(host, mrq);
		}
	}
}

static void spsdc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct spsdc_host *host = mmc_priv(mmc);
	int ret;

	if (ios->power_mode == MMC_POWER_OFF) {
		/* No register access: the next power-on resets the controller. */
		mmc_regulator_disable_vqmmc(mmc);
		mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);
		if (host->pm_ref) {
			pm_runtime_put_autosuspend(mmc_dev(mmc));
			host->pm_ref = false;
		}
		host->power_state = MMC_POWER_OFF;
		return;
	}
	/* power_state changes only when a mode is fully set up, so a failed step is retried. */
	if (ios->power_mode != host->power_state) {
		if (mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, ios->vdd))
			return;
		if (ios->power_mode == MMC_POWER_ON && mmc_regulator_enable_vqmmc(mmc))
			return;
	}
	/* The controller clock must be on before the first register access. */
	if (!host->pm_ref) {
		ret = pm_runtime_resume_and_get(mmc_dev(mmc));
		if (ret) {
			dev_err(mmc_dev(mmc), "failed to resume: %d\n", ret);
			return;
		}
		host->pm_ref = true;
	}
	/* Unclocked after a failed resume, or gated on a broken controller. */
	if (!host->clk_enabled)
		return;
	if (ios->power_mode == MMC_POWER_ON && host->power_state != MMC_POWER_ON &&
	    spsdc_controller_init(host))
		return;
	host->power_state = ios->power_mode;

	spsdc_set_bus_clk(host, ios->clock);
	spsdc_set_bus_timing(host, ios->timing);
	spsdc_set_bus_width(host, ios->bus_width);
	/* An unknown width stays unknown until a request sets it again. */
	if (host->sdio_width != SPSDC_SDIO_WIDTH_UNKNOWN)
		host->sdio_width = ios->bus_width;
	spsdc_select_mode(host);
}

static void spsdc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct spsdc_host *host = mmc_priv(mmc);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	/* Unclocked after a failed resume, or gated on a broken controller. */
	if (host->clk_enabled)
		__spsdc_enable_sdio_irq(host, enable);
	spin_unlock_irqrestore(&host->lock, flags);
}

static void spsdc_ack_sdio_irq(struct mmc_host *mmc)
{
	spsdc_enable_sdio_irq(mmc, 1);
}

static const struct mmc_host_ops spsdc_ops = {
	.request         = spsdc_request,
	.set_ios         = spsdc_set_ios,
	.get_cd          = mmc_gpio_get_cd,
	.get_ro          = mmc_gpio_get_ro,
	.enable_sdio_irq = spsdc_enable_sdio_irq,
	.ack_sdio_irq    = spsdc_ack_sdio_irq,
};

static int spsdc_drv_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct spsdc_host *host;
	int ret;

	mmc = devm_mmc_alloc_host(&pdev->dev, sizeof(*host));
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->power_state = MMC_POWER_OFF;
	spin_lock_init(&host->lock);
	INIT_DELAYED_WORK(&host->timeout_work, spsdc_request_timeout);

	host->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(host->base))
		return PTR_ERR(host->base);

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0)
		return host->irq;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to set DMA mask\n");

	host->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(host->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(host->clk),
				     "failed to get clock\n");

	host->rstc = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(host->rstc))
		return dev_err_probe(&pdev->dev, PTR_ERR(host->rstc),
				     "failed to get reset\n");

	ret = mmc_regulator_get_supply(mmc);
	if (ret)
		return ret;

	ret = clk_prepare_enable(host->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable clock: %d\n", ret);
		return ret;
	}
	host->clk_enabled = true;
	if (!clk_get_rate(host->clk)) {
		ret = dev_err_probe(&pdev->dev, -EINVAL, "controller clock rate is 0\n");
		goto disable_clk;
	}

	ret = reset_control_deassert(host->rstc);
	if (ret) {
		dev_err(&pdev->dev, "failed to deassert reset: %d\n", ret);
		goto disable_clk;
	}

	ret = mmc_of_parse(mmc);
	if (ret)
		goto assert_reset;

	mmc->ops = &spsdc_ops;
	mmc->f_min = SPSDC_MIN_CLK;
	if (!mmc->f_max || mmc->f_max > SPSDC_MAX_CLK)
		mmc->f_max = SPSDC_MAX_CLK;
	if (!mmc->ocr_avail)
		mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->max_seg_size = SPSDC_MAX_BLK_COUNT * SPSDC_PAGE_SIZE;
	mmc->max_segs = SPSDC_MAX_DMA_MEMORY_SECTORS;
	mmc->max_req_size = SPSDC_MAX_BLK_COUNT * SPSDC_PAGE_SIZE;
	mmc->max_blk_size = SPSDC_PAGE_SIZE;
	mmc->max_blk_count = SPSDC_MAX_BLK_COUNT;

	if ((mmc->caps2 & MMC_CAP2_NO_SD) && (mmc->caps2 & MMC_CAP2_NO_MMC)) {
		host->mode = SPSDC_MODE_SDIO;
		host->bounce_buf = devm_kmalloc(&pdev->dev, SPSDC_SDIO_BOUNCE_SIZE,
						GFP_KERNEL);
		if (!host->bounce_buf) {
			ret = -ENOMEM;
			goto assert_reset;
		}
		mmc->max_segs = SPSDC_SDIO_MAX_SEGS;
		mmc->max_seg_size = SPSDC_SDIO_BOUNCE_SIZE;
		mmc->max_req_size = SPSDC_SDIO_BOUNCE_SIZE;
		if ((mmc->caps & MMC_CAP_SDIO_IRQ) && (mmc->caps & MMC_CAP_4_BIT_DATA)) {
			dev_warn(&pdev->dev,
				 "cap-sdio-irq ignored with a 4-bit bus: card interrupts are only seen at the next transaction\n");
			mmc->caps &= ~MMC_CAP_SDIO_IRQ;
		}
		if (mmc->caps & MMC_CAP_SDIO_IRQ)
			mmc->caps2 |= MMC_CAP2_SDIO_IRQ_NOTHREAD;
	} else {
		host->mode = SPSDC_MODE_SD;
		/* Only the SDIO-mode set-up can handle SDIO cards. */
		mmc->caps2 |= MMC_CAP2_NO_SDIO;
	}

	dev_set_drvdata(&pdev->dev, host);
	ret = spsdc_controller_init(host);
	if (ret)
		goto assert_reset;
	spsdc_select_mode(host);

	ret = devm_request_threaded_irq(&pdev->dev, host->irq, spsdc_irq,
					spsdc_irq_thread, IRQF_ONESHOT,
					dev_name(&pdev->dev), host);
	if (ret)
		goto assert_reset;

	pm_runtime_set_autosuspend_delay(&pdev->dev, SPSDC_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = mmc_add_host(mmc);
	if (ret)
		goto disable_pm;
	return 0;

disable_pm:
	disable_irq(host->irq);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_dont_use_autosuspend(&pdev->dev);
assert_reset:
	reset_control_assert(host->rstc);
disable_clk:
	clk_disable_unprepare(host->clk);
	return ret;
}

static void spsdc_drv_remove(struct platform_device *pdev)
{
	struct spsdc_host *host = platform_get_drvdata(pdev);

	mmc_remove_host(host->mmc);
	cancel_delayed_work_sync(&host->timeout_work);
	/* The devm IRQ outlives remove(); keep it off the gated controller. */
	disable_irq(host->irq);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	reset_control_assert(host->rstc);
	if (host->clk_enabled)
		clk_disable(host->clk);
	clk_unprepare(host->clk);
}

static int spsdc_runtime_suspend(struct device *dev)
{
	struct spsdc_host *host = dev_get_drvdata(dev);

	if (!host->clk_enabled)
		return 0;
	/* Keep the line masked while unclocked; this also waits for running handlers. */
	disable_irq(host->irq);
	cancel_delayed_work_sync(&host->timeout_work);
	clk_disable(host->clk);
	host->clk_enabled = false;
	return 0;
}

static int spsdc_runtime_resume(struct device *dev)
{
	struct spsdc_host *host = dev_get_drvdata(dev);
	int ret;

	/* A clock on again could restart a DMA that could not be stopped. */
	if (host->broken)
		return 0;
	if (!host->clk_enabled) {
		ret = clk_enable(host->clk);
		if (ret)
			return ret;
		host->clk_enabled = true;
		enable_irq(host->irq);
	}
	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(spsdc_pm_ops, spsdc_runtime_suspend,
				 spsdc_runtime_resume, NULL);

static const struct of_device_id spsdc_of_table[] = {
	{ .compatible = "sunplus,sp7021-sd" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, spsdc_of_table);

static struct platform_driver spsdc_driver = {
	.probe  = spsdc_drv_probe,
	.remove = spsdc_drv_remove,
	.driver = {
		.name           = "sunplus-sd",
		.pm             = pm_ptr(&spsdc_pm_ops),
		.of_match_table = spsdc_of_table,
	},
};
module_platform_driver(spsdc_driver);

MODULE_AUTHOR("Li-hao Kuo <lhjeff911@gmail.com>");
MODULE_DESCRIPTION("Sunplus SP7021 SD/SDIO host controller driver");
MODULE_LICENSE("GPL");
