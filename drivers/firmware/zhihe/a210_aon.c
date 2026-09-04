// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 Zhihe Computing Limited.
 */

#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/firmware/zhihe/a210-ipc.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#define ZHIHE_A210_AON_SYNC_VAL		0xfeabdeff
#define ZHIHE_A210_AON_RESET_VAL	0x3

#define ZHIHE_A210_AON_RX_TIMEOUT	msecs_to_jiffies(3000)

struct zhihe_a210_aon_chan {
	struct zhihe_a210_aon_ipc *aon_ipc;
	struct mbox_client cl;
	struct mbox_chan *ch;
};

struct zhihe_a210_aon_ipc {
	struct zhihe_a210_aon_chan chans;
	struct device *dev;
	struct mutex lock; /* Serializes RPC transactions. */
	struct completion done;
	void __iomem *firmware_base;
	void __iomem *reset_reg;
	void __iomem *sync_reg;
	resource_size_t firmware_size;
	void __iomem *log_base;
	resource_size_t log_size;
	struct mutex log_lock; /* Serializes firmware log readers. */
	u32 *msg;
	size_t msg_size;
};

struct zhihe_a210_aon_log {
	u32 panic;
	u32 reserved[2];
	u32 read;
	u32 write;
	u32 size;
	u32 ring_reserved;
	u8 data[];
};

/*
 * This type is used to indicate error response for most functions.
 */
enum zhihe_a210_aon_error_codes {
	ZHIHE_A210_AON_ERR_NONE = 0, /* Success */
	ZHIHE_A210_AON_ERR_VERSION = 1, /* Incompatible API version */
	ZHIHE_A210_AON_ERR_CONFIG = 2, /* Configuration error */
	ZHIHE_A210_AON_ERR_PARM = 3, /* Bad parameter */
	ZHIHE_A210_AON_ERR_NOACCESS = 4, /* Permission error (no access) */
	ZHIHE_A210_AON_ERR_LOCKED = 5, /* Permission error (locked) */
	ZHIHE_A210_AON_ERR_UNAVAILABLE = 6, /* Unavailable (out of resources) */
	ZHIHE_A210_AON_ERR_NOTFOUND = 7, /* Not found */
	ZHIHE_A210_AON_ERR_NOPOWER = 8, /* No power */
	ZHIHE_A210_AON_ERR_IPC = 9, /* Generic IPC error */
	ZHIHE_A210_AON_ERR_BUSY = 10, /* Resource is currently busy/active */
	ZHIHE_A210_AON_ERR_FAIL = 11, /* General I/O failure */
	ZHIHE_A210_AON_ERR_LAST
};

static const int zhihe_a210_aon_linux_errmap[ZHIHE_A210_AON_ERR_LAST] = {
	0, /* ZHIHE_A210_AON_ERR_NONE */
	-EINVAL, /* ZHIHE_A210_AON_ERR_VERSION */
	-EINVAL, /* ZHIHE_A210_AON_ERR_CONFIG */
	-EINVAL, /* ZHIHE_A210_AON_ERR_PARM */
	-EACCES, /* ZHIHE_A210_AON_ERR_NOACCESS */
	-EACCES, /* ZHIHE_A210_AON_ERR_LOCKED */
	-ENOSPC, /* ZHIHE_A210_AON_ERR_UNAVAILABLE */
	-ENOENT, /* ZHIHE_A210_AON_ERR_NOTFOUND */
	-EPERM, /* ZHIHE_A210_AON_ERR_NOPOWER */
	-EPIPE, /* ZHIHE_A210_AON_ERR_IPC */
	-EBUSY, /* ZHIHE_A210_AON_ERR_BUSY */
	-EIO, /* ZHIHE_A210_AON_ERR_FAIL */
};

static int zhihe_a210_aon_to_linux_errno(int errno)
{
	if (errno < 0)
		return errno;
	if (errno >= ZHIHE_A210_AON_ERR_NONE && errno < ZHIHE_A210_AON_ERR_LAST)
		return zhihe_a210_aon_linux_errmap[errno];
	return -EIO;
}

static void zhihe_a210_aon_free_channel(void *data)
{
	mbox_free_channel(data);
}

static void zhihe_a210_aon_rx_callback(struct mbox_client *c, void *msg)
{
	struct zhihe_a210_aon_chan *aon_chan = container_of(c, struct zhihe_a210_aon_chan, cl);
	struct zhihe_a210_aon_ipc *aon_ipc = aon_chan->aon_ipc;
	size_t recv_size = ZHIHE_A210_AON_RPC_MSG_NUM * sizeof(u32);
	size_t copy_size;

	if (!msg)
		return;

	copy_size = min(recv_size, aon_ipc->msg_size);
	if (copy_size)
		memcpy(aon_ipc->msg, msg, copy_size);
	dev_dbg(aon_ipc->dev, "msg head: 0x%x, size:%zu\n", *((u32 *)msg),
		recv_size);
	complete(&aon_ipc->done);
}

static int zhihe_a210_aon_ipc_write(struct zhihe_a210_aon_ipc *aon_ipc, void *msg)
{
	struct zhihe_a210_aon_rpc_msg_hdr *hdr = msg;
	struct zhihe_a210_aon_chan *aon_chan;
	u32 *data = msg;
	int ret;
	/* check size, currently it requires 7 MSG in one transfer */
	if (hdr->size != ZHIHE_A210_AON_RPC_MSG_NUM)
		return -EINVAL;

	dev_dbg(aon_ipc->dev, "RPC SVC %u FUNC %u SIZE %u\n", hdr->svc,
		hdr->func, hdr->size);

	aon_chan = &aon_ipc->chans;

	ret = mbox_send_message(aon_chan->ch, data);
	if (ret < 0)
		return ret;

	return 0;
}

/*
 * RPC command/response
 */
int zhihe_a210_aon_call_rpc(struct zhihe_a210_aon_ipc *aon_ipc,
			    void *msg, void *ack_msg,
		       size_t ack_size, bool have_resp)
{
	struct zhihe_a210_aon_rpc_msg_hdr *hdr = msg;
	struct zhihe_a210_aon_rpc_ack_common *ack;
	unsigned long timeout = ZHIHE_A210_AON_RX_TIMEOUT;
	int ret = 0;

	if (WARN_ON(!aon_ipc || !msg))
		return -EINVAL;
	if (have_resp && WARN_ON(!ack_msg))
		return -EINVAL;
	if (have_resp && WARN_ON(ack_size < sizeof(*ack)))
		return -EINVAL;
	mutex_lock(&aon_ipc->lock);
	reinit_completion(&aon_ipc->done);
	aon_ipc->msg = ack_msg;
	aon_ipc->msg_size = ack_size;
	ZHIHE_A210_RPC_SET_VER(hdr, ZHIHE_A210_AON_RPC_VERSION);
	/*svc id use 6bit for version 2*/
	ZHIHE_A210_RPC_SET_SVC_ID(hdr, hdr->svc);
	ZHIHE_A210_RPC_SET_SVC_FLAG_MSG_TYPE(hdr, ZHIHE_A210_RPC_SVC_MSG_TYPE_DATA);
	if (have_resp)
		ZHIHE_A210_RPC_SET_SVC_FLAG_ACK_TYPE(hdr, ZHIHE_A210_RPC_SVC_MSG_NEED_ACK);
	else
		ZHIHE_A210_RPC_SET_SVC_FLAG_ACK_TYPE(hdr, ZHIHE_A210_RPC_SVC_MSG_NO_NEED_ACK);
	ret = zhihe_a210_aon_ipc_write(aon_ipc, msg);
	if (ret < 0) {
		dev_err(aon_ipc->dev, "RPC send msg failed: %d\n", ret);
		goto out;
	}
	if (have_resp) {
		if (!wait_for_completion_timeout(&aon_ipc->done, timeout)) {
			dev_err(aon_ipc->dev, "RPC send msg timeout\n");
			aon_ipc->msg = NULL;
			aon_ipc->msg_size = 0;
			mutex_unlock(&aon_ipc->lock);
			return -ETIMEDOUT;
		}

		/* response status is stored in msg data[0] field */
		ack = ack_msg;
		ret = ack->err_code;
	}
out:
	aon_ipc->msg = NULL;
	aon_ipc->msg_size = 0;
	mutex_unlock(&aon_ipc->lock);

	dev_dbg(aon_ipc->dev, "RPC SVC done\n");

	return zhihe_a210_aon_to_linux_errno(ret);
}
EXPORT_SYMBOL(zhihe_a210_aon_call_rpc);

static int zhihe_a210_aon_map_resources(struct platform_device *pdev,
					struct zhihe_a210_aon_ipc *aon_ipc)
{
	struct device *dev = &pdev->dev;
	struct resource *resource;

	resource = platform_get_resource_byname(pdev, IORESOURCE_MEM, "aon-base");
	if (!resource)
		return dev_err_probe(dev, -EINVAL,
				     "missing AON firmware memory resource\n");
	aon_ipc->firmware_size = resource_size(resource);
	aon_ipc->firmware_base = devm_ioremap_resource(dev, resource);
	if (IS_ERR(aon_ipc->firmware_base))
		return dev_err_probe(dev, PTR_ERR(aon_ipc->firmware_base),
				     "failed to map AON firmware memory\n");

	aon_ipc->reset_reg =
		devm_platform_ioremap_resource_byname(pdev, "aon-reset");
	if (IS_ERR(aon_ipc->reset_reg))
		return dev_err_probe(dev, PTR_ERR(aon_ipc->reset_reg),
				     "failed to map AON reset register\n");

	aon_ipc->sync_reg =
		devm_platform_ioremap_resource_byname(pdev, "aon-sync");
	if (IS_ERR(aon_ipc->sync_reg))
		return dev_err_probe(dev, PTR_ERR(aon_ipc->sync_reg),
				     "failed to map AON sync register\n");

	return 0;
}

static int zhihe_a210_aon_boot_firmware(struct platform_device *pdev,
					struct zhihe_a210_aon_ipc *aon_ipc)
{
	struct device *dev = &pdev->dev;
	const struct firmware *aon_fw;
	const char *firmware_name;
	unsigned int sync;
	int attempt;
	int ret;

	ret = device_property_read_string(dev, "firmware-name", &firmware_name);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to get firmware-name property\n");

	ret = request_firmware(&aon_fw, firmware_name, dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to load %s\n",
				     firmware_name);

	if (aon_fw->size > aon_ipc->firmware_size) {
		dev_err(dev, "firmware is too large: %zu > %pa\n",
			aon_fw->size, &aon_ipc->firmware_size);
		ret = -EFBIG;
		goto release_firmware;
	}

	for (attempt = 0; attempt < 2; attempt++) {
		memset_io(aon_ipc->firmware_base, 0, aon_ipc->firmware_size);
		memcpy_toio(aon_ipc->firmware_base, aon_fw->data, aon_fw->size);
		iowrite32(ZHIHE_A210_AON_RESET_VAL, aon_ipc->reset_reg);

		ret = readl_poll_timeout(aon_ipc->sync_reg, sync,
					 sync == ZHIHE_A210_AON_SYNC_VAL,
					 1000, 50000);
		if (!ret)
			break;
		dev_warn(dev, "AON startup timed out, retrying\n");
	}
	if (ret)
		dev_err(dev, "AON startup failed: sync %#x, expected %#x\n",
			sync, ZHIHE_A210_AON_SYNC_VAL);

release_firmware:
	release_firmware(aon_fw);

	return ret;
}

static int zhihe_a210_aon_log_show(struct seq_file *seq, void *data)
{
	struct device *dev = data;
	struct zhihe_a210_aon_ipc *aon_ipc = dev_get_drvdata(dev);
	struct zhihe_a210_aon_log __iomem *log = aon_ipc->log_base;
	resource_size_t capacity = aon_ipc->log_size - offsetof(typeof(*log), data);
	u8 buffer[64];
	u32 remaining;
	u32 write;
	u32 read;
	u32 size;
	u32 panic;

	mutex_lock(&aon_ipc->log_lock);
	panic = readl(&log->panic);
	read = readl(&log->read);
	write = readl(&log->write);
	size = readl(&log->size);

	if (!size || size > capacity || read >= size || write >= size) {
		mutex_unlock(&aon_ipc->log_lock);
		return -EIO;
	}

	remaining = write >= read ? write - read : size - read + write;
	seq_printf(seq, "panic: %#x\n", panic);

	while (remaining) {
		size_t chunk = min_t(size_t, remaining, sizeof(buffer));

		chunk = min_t(size_t, chunk, size - read);
		memcpy_fromio(buffer, &log->data[read], chunk);
		seq_printf(seq, "%*pE", (int)chunk, buffer);
		read = (read + chunk) % size;
		remaining -= chunk;
	}
	seq_putc(seq, '\n');

	/* Consuming the snapshot lets the firmware reuse the ring space. */
	writel(write, &log->read);
	mutex_unlock(&aon_ipc->log_lock);

	return 0;
}

static void zhihe_a210_aon_debugfs_remove(void *data)
{
	debugfs_remove_recursive(data);
}

static int a210_aon_debugfs_init(struct device *dev, struct zhihe_a210_aon_ipc *aon_ipc)
{
	struct device_node *memory_node;
	struct reserved_mem *reserved_mem;
	struct dentry *directory;
	int index;
	int ret;

	if (!IS_ENABLED(CONFIG_DEBUG_FS))
		return 0;

	index = of_property_match_string(dev->of_node, "memory-region-names",
					 "log");
	if (index == -EINVAL || index == -ENODATA)
		return 0;
	if (index < 0)
		return dev_err_probe(dev, index,
				     "failed to find AON log memory region\n");

	memory_node = of_parse_phandle(dev->of_node, "memory-region", index);
	if (!memory_node)
		return dev_err_probe(dev, -EINVAL,
				     "failed to parse AON log memory region\n");

	reserved_mem = of_reserved_mem_lookup(memory_node);
	of_node_put(memory_node);
	if (!reserved_mem)
		return dev_err_probe(dev, -EINVAL,
				     "failed to look up AON log memory region\n");
	if (reserved_mem->size <= offsetof(struct zhihe_a210_aon_log, data))
		return dev_err_probe(dev, -EINVAL,
				     "AON log memory region is too small\n");

	aon_ipc->log_base = devm_ioremap(dev, reserved_mem->base,
					 reserved_mem->size);
	if (!aon_ipc->log_base)
		return -ENOMEM;
	aon_ipc->log_size = reserved_mem->size;
	mutex_init(&aon_ipc->log_lock);

	directory = debugfs_create_dir("zhihe-a210-aon", NULL);
	if (IS_ERR(directory))
		return 0;

	ret = devm_add_action_or_reset(dev, zhihe_a210_aon_debugfs_remove,
				       directory);
	if (ret)
		return ret;

	debugfs_create_devm_seqfile(dev, "firmware_log", directory,
				    zhihe_a210_aon_log_show);

	return 0;
}

static int zhihe_a210_aon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zhihe_a210_aon_ipc *aon_ipc;
	struct zhihe_a210_aon_chan *aon_chan;
	struct mbox_client *cl;
	int ret;

	aon_ipc = devm_kzalloc(dev, sizeof(*aon_ipc), GFP_KERNEL);
	if (!aon_ipc)
		return -ENOMEM;

	ret = zhihe_a210_aon_map_resources(pdev, aon_ipc);
	if (ret)
		return ret;

	aon_chan = &aon_ipc->chans;
	cl = &aon_chan->cl;
	cl->dev = dev;
	cl->tx_block = true;
	cl->tx_tout = 500;
	cl->rx_callback = zhihe_a210_aon_rx_callback;

	aon_chan->aon_ipc = aon_ipc;
	aon_chan->ch = mbox_request_channel(cl, 0);
	if (IS_ERR(aon_chan->ch)) {
		ret = PTR_ERR(aon_chan->ch);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to request aon mbox chan ret %d\n",
				ret);
		return ret;
	}
	ret = devm_add_action_or_reset(dev, zhihe_a210_aon_free_channel,
				       aon_chan->ch);
	if (ret)
		return ret;

	aon_ipc->dev = dev;
	mutex_init(&aon_ipc->lock);
	init_completion(&aon_ipc->done);

	ret = zhihe_a210_aon_boot_firmware(pdev, aon_ipc);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, aon_ipc);
	ret = a210_aon_debugfs_init(dev, aon_ipc);
	if (ret)
		return ret;

	return devm_of_platform_populate(dev);
}

static const struct of_device_id zhihe_a210_aon_match[] = {
	{
		.compatible = "zhihe,a210-aon",
	},
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, zhihe_a210_aon_match);

static struct platform_driver zhihe_a210_aon_driver = {
	.driver = {
		.name = "zhihe-a210-aon",
		.of_match_table = zhihe_a210_aon_match,
	},
	.probe = zhihe_a210_aon_probe,
};
module_platform_driver(zhihe_a210_aon_driver);

MODULE_AUTHOR("Hongkun Xu <xuhongkun@zhcomputing.com>");
MODULE_AUTHOR("Xionglue Huang <huangxionglue@zhcomputing.com>");
MODULE_DESCRIPTION("ZhiHe A210 AON firmware driver");
MODULE_FIRMWARE("a210-aon.bin");
MODULE_LICENSE("GPL");
