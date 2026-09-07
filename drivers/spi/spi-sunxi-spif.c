// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026 Han Gao <gaohan@iscas.ac.cn> */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>

#define SPIF_VERSION		0x00
#define SPIF_GCR			0x04
#define SPIF_GAR			0x08
#define SPIF_TCR			0x0c
#define SPIF_IER			0x14
#define SPIF_ISR			0x18
#define SPIF_CSD			0x1c
#define SPIF_DMA_CTL		0x40
#define SPIF_DMA_DESC		0x44
#define SPIF_DMA_MODE		BIT(0)
#define SPIF_CPHA		BIT(4)
#define SPIF_CPOL		BIT(5)
#define SPIF_CS			GENMASK(7, 6)
#define SPIF_CS_ACTIVE_LOW	BIT(8)
#define SPIF_FIFO_RESET		GENMASK(1, 0)
#define SPIF_SOFT_RESET		BIT(3)
#define SPIF_DMA_RESET		BIT(4)
#define SPIF_DMA_DONE		BIT(24)
#define SPIF_FIFO_ERRORS		GENMASK(10, 8)
#define SPIF_IRQS		(SPIF_DMA_DONE | SPIF_FIFO_ERRORS)
#define SPIF_PHASE_RX		BIT(8)
#define SPIF_PHASE_TX		BIT(12)
#define SPIF_PHASE_DUMMY		BIT(16)
#define SPIF_PHASE_ADDR		BIT(24)
#define SPIF_PHASE_CMD		BIT(28)
#define SPIF_DESC_LAST		BIT(0)
#define SPIF_DESC_READ		BIT(1)
#define SPIF_DESC_BURST		GENMASK(6, 4)
#define SPIF_DESC_BLOCK		GENMASK(25, 24)
#define SPIF_DESC_ADDR_LEN	GENMASK(25, 24)
#define SPIF_DESC_DUMMY_BITS	GENMASK(23, 16)
#define SPIF_DESC_NORMAL		BIT(28)
#define SPIF_DMA_START		BIT(0)
#define SPIF_DMA_DESC_LEN		GENMASK(15, 4)
#define SPIF_BUFFER_SIZE		4096

struct sunxi_spif_desc {
	__le32 burst;
	__le32 block_len;
	__le32 data_addr;
	__le32 next_desc;
	__le32 phase;
	__le32 flash_addr;
	__le32 bus_width;
	__le32 transfer_len;
};

struct sunxi_spif {
	struct device *dev;
	void __iomem *base;
	struct clk *bus_clk;
	struct clk *mod_clk;
	struct reset_control *rst;
	struct completion done;
	struct sunxi_spif_desc *desc;
	u8 *buffer;
	u32 status;
	unsigned int speed;
	int irq;
	bool clocks_on;
	bool failed;
};

static int sunxi_spif_reset(struct sunxi_spif *spif)
{
	u32 value;
	int ret;

	writel(readl(spif->base + SPIF_GAR) | SPIF_DMA_RESET,
	       spif->base + SPIF_GAR);
	ret = readl_poll_timeout(spif->base + SPIF_GAR, value,
				 !(value & SPIF_DMA_RESET), 1, 100000);
	if (ret)
		return ret;
	writel(readl(spif->base + SPIF_GAR) | SPIF_SOFT_RESET,
	       spif->base + SPIF_GAR);
	return readl_poll_timeout(spif->base + SPIF_GAR, value,
				  !(value & SPIF_SOFT_RESET), 1, 100000);
}

static void sunxi_spif_disable(void *data)
{
	struct sunxi_spif *spif = data;

	if (!spif->clocks_on)
		return;
	writel(0, spif->base + SPIF_IER);
	reset_control_assert(spif->rst);
	clk_disable_unprepare(spif->mod_clk);
	clk_disable_unprepare(spif->bus_clk);
	spif->clocks_on = false;
}

static irqreturn_t sunxi_spif_irq(int irq, void *data)
{
	struct sunxi_spif *spif = data;
	u32 status = readl(spif->base + SPIF_ISR) & readl(spif->base + SPIF_IER);

	if (!(status & SPIF_IRQS))
		return IRQ_NONE;
	writel(0, spif->base + SPIF_IER);
	writel(status, spif->base + SPIF_ISR);
	spif->status = status;
	complete(&spif->done);
	return IRQ_HANDLED;
}

static bool sunxi_spif_supports_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	if (op->cmd.nbytes != 1 || op->cmd.buswidth != 1 ||
	    op->addr.nbytes > 4 || op->addr.val > U32_MAX ||
	    (op->addr.nbytes && op->addr.buswidth != 1) ||
	    op->dummy.nbytes > 31 ||
	    (op->dummy.nbytes && op->dummy.buswidth != 1) ||
	    (op->data.nbytes && op->data.buswidth != 1))
		return false;

	return spi_mem_default_supports_op(mem, op);
}

static int sunxi_spif_adjust_op_size(struct spi_mem *mem, struct spi_mem_op *op)
{
	op->data.nbytes = min_t(unsigned int, op->data.nbytes, SPIF_BUFFER_SIZE);
	return 0;
}

static int sunxi_spif_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct sunxi_spif *spif = spi_controller_get_devdata(mem->spi->controller);
	struct sunxi_spif_desc *desc = spif->desc;
	enum dma_data_direction direction = op->data.dir == SPI_MEM_DATA_IN ?
		DMA_FROM_DEVICE : DMA_TO_DEVICE;
	dma_addr_t buffer = 0, descriptor;
	unsigned int speed = mem->spi->max_speed_hz;
	u32 burst, phase, transfer, control;
	int ret;

	if (spif->failed)
		return -EIO;
	if (!sunxi_spif_supports_op(mem, op) || op->data.nbytes > SPIF_BUFFER_SIZE)
		return -EOPNOTSUPP;
	if (op->max_freq)
		speed = min(speed, op->max_freq);
	if (!speed)
		return -EINVAL;
	speed = min(speed, 50000000U);
	if (speed != spif->speed) {
		ret = clk_set_rate(spif->mod_clk, speed);
		if (ret)
			return ret;
		if (clk_get_rate(spif->mod_clk) > speed)
			return -ERANGE;
		spif->speed = speed;
	}

	memset(desc, 0, sizeof(*desc));
	burst = SPIF_DESC_LAST | FIELD_PREP(SPIF_DESC_BURST, 7);
	phase = SPIF_PHASE_CMD;
	transfer = SPIF_DESC_NORMAL | op->data.nbytes;
	desc->block_len = cpu_to_le32(FIELD_PREP(SPIF_DESC_BLOCK, 3) | op->data.nbytes);
	desc->bus_width = cpu_to_le32((u32)op->cmd.opcode << 24);
	if (op->addr.nbytes) {
		phase |= SPIF_PHASE_ADDR;
		desc->flash_addr = cpu_to_le32(op->addr.val);
		transfer |= FIELD_PREP(SPIF_DESC_ADDR_LEN, op->addr.nbytes - 1);
	}
	if (op->dummy.nbytes) {
		phase |= SPIF_PHASE_DUMMY;
		transfer |= FIELD_PREP(SPIF_DESC_DUMMY_BITS, op->dummy.nbytes * 8);
	}
	if (op->data.nbytes) {
		if (op->data.dir == SPI_MEM_DATA_IN) {
			phase |= SPIF_PHASE_RX;
			burst |= SPIF_DESC_READ;
		} else {
			phase |= SPIF_PHASE_TX;
			memcpy(spif->buffer, op->data.buf.out, op->data.nbytes);
		}
		buffer = dma_map_single(spif->dev, spif->buffer, op->data.nbytes, direction);
		if (dma_mapping_error(spif->dev, buffer))
			return -ENOMEM;
		desc->data_addr = cpu_to_le32(buffer >> 2);
	}
	desc->burst = cpu_to_le32(burst);
	desc->phase = cpu_to_le32(phase);
	desc->transfer_len = cpu_to_le32(transfer);
	descriptor = dma_map_single(spif->dev, desc, sizeof(*desc), DMA_TO_DEVICE);
	if (dma_mapping_error(spif->dev, descriptor)) {
		ret = -ENOMEM;
		goto unmap_buffer;
	}

	control = SPIF_DMA_MODE | SPIF_CS_ACTIVE_LOW |
		  FIELD_PREP(SPIF_CS, spi_get_chipselect(mem->spi, 0));
	if (mem->spi->mode & SPI_CPHA)
		control |= SPIF_CPHA;
	if (mem->spi->mode & SPI_CPOL)
		control |= SPIF_CPOL;
	reinit_completion(&spif->done);
	spif->status = 0;
	writel(readl(spif->base + SPIF_GAR) | SPIF_FIFO_RESET, spif->base + SPIF_GAR);
	writel(control, spif->base + SPIF_GCR);
	writel(SPIF_IRQS, spif->base + SPIF_ISR);
	writel(descriptor >> 2, spif->base + SPIF_DMA_DESC);
	writel(SPIF_IRQS, spif->base + SPIF_IER);
	/* Make the descriptor visible before the device starts fetching it. */
	dma_wmb();
	writel(FIELD_PREP(SPIF_DMA_DESC_LEN, sizeof(*desc)) | SPIF_DMA_START,
	       spif->base + SPIF_DMA_CTL);

	if (!wait_for_completion_timeout(&spif->done, msecs_to_jiffies(1000)))
		ret = -ETIMEDOUT;
	else
		ret = spif->status & SPIF_FIFO_ERRORS ? -EIO : 0;
	writel(0, spif->base + SPIF_IER);
	synchronize_irq(spif->irq);
	if (ret && sunxi_spif_reset(spif)) {
		/* Do not hand a live DMA buffer back to the caller. */
		sunxi_spif_disable(spif);
		spif->failed = true;
	}
	dma_unmap_single(spif->dev, descriptor, sizeof(*desc), DMA_TO_DEVICE);
unmap_buffer:
	if (op->data.nbytes)
		dma_unmap_single(spif->dev, buffer, op->data.nbytes, direction);
	if (!ret && op->data.nbytes && op->data.dir == SPI_MEM_DATA_IN)
		memcpy(op->data.buf.in, spif->buffer, op->data.nbytes);
	return ret;
}

static const struct spi_controller_mem_ops sunxi_spif_mem_ops = {
	.supports_op = sunxi_spif_supports_op,
	.adjust_op_size = sunxi_spif_adjust_op_size,
	.exec_op = sunxi_spif_exec_op,
};

static int sunxi_spif_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *ctlr;
	struct sunxi_spif *spif;
	int ret;

	ctlr = devm_spi_alloc_host(dev, sizeof(*spif));
	if (!ctlr)
		return -ENOMEM;
	spif = spi_controller_get_devdata(ctlr);
	spif->dev = dev;
	spif->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spif->base))
		return PTR_ERR(spif->base);
	spif->bus_clk = devm_clk_get(dev, "ahb");
	if (IS_ERR(spif->bus_clk))
		return dev_err_probe(dev, PTR_ERR(spif->bus_clk), "getting bus clock\n");
	spif->mod_clk = devm_clk_get(dev, "mod");
	if (IS_ERR(spif->mod_clk))
		return dev_err_probe(dev, PTR_ERR(spif->mod_clk), "getting module clock\n");
	spif->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(spif->rst))
		return dev_err_probe(dev, PTR_ERR(spif->rst), "getting reset\n");
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	/* Separate cacheline-aligned allocations avoid DMA ownership sharing. */
	spif->desc = devm_kzalloc(dev, max_t(size_t, sizeof(*spif->desc),
					   dma_get_cache_alignment()), GFP_KERNEL);
	spif->buffer = devm_kmalloc(dev, SPIF_BUFFER_SIZE, GFP_KERNEL);
	if (!spif->desc || !spif->buffer)
		return -ENOMEM;
	init_completion(&spif->done);
	spif->irq = platform_get_irq(pdev, 0);
	if (spif->irq < 0)
		return spif->irq;
	ret = clk_prepare_enable(spif->bus_clk);
	if (ret)
		return ret;
	ret = clk_prepare_enable(spif->mod_clk);
	if (ret) {
		clk_disable_unprepare(spif->bus_clk);
		return ret;
	}
	spif->clocks_on = true;
	ret = devm_add_action_or_reset(dev, sunxi_spif_disable, spif);
	if (ret)
		return ret;
	ret = reset_control_deassert(spif->rst);
	if (ret)
		return ret;
	writel(0, spif->base + SPIF_IER);
	ret = sunxi_spif_reset(spif);
	if (ret)
		return ret;
	if (readl(spif->base + SPIF_VERSION) < 0x10002)
		return -ENODEV;
	writel(0, spif->base + SPIF_TCR);
	writel((5 << 16) | (6 << 8) | 6, spif->base + SPIF_CSD);
	ret = devm_request_irq(dev, spif->irq, sunxi_spif_irq, 0, dev_name(dev), spif);
	if (ret)
		return ret;
	ctlr->mode_bits = SPI_CPOL | SPI_CPHA;
	ctlr->num_chipselect = 4;
	ctlr->max_speed_hz = 50000000;
	ctlr->mem_ops = &sunxi_spif_mem_ops;
	ctlr->dev.of_node = dev->of_node;
	return devm_spi_register_controller(dev, ctlr);
}

static const struct of_device_id sunxi_spif_match[] = {
	{ .compatible = "allwinner,sun252i-v861-spif" },
	{ }
};
MODULE_DEVICE_TABLE(of, sunxi_spif_match);

static struct platform_driver sunxi_spif_driver = {
	.probe = sunxi_spif_probe,
	.driver = {
		.name = "sunxi-spif",
		.of_match_table = sunxi_spif_match,
	},
};
module_platform_driver(sunxi_spif_driver);

MODULE_DESCRIPTION("Allwinner dedicated SPI flash controller");
MODULE_LICENSE("GPL");
