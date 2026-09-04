// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 Zhihe Computing Limited.
 */

#include <linux/err.h>
#include <linux/firmware/zhihe/a210-ipc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#define MBOX_MAX_MSG_LEN	28

/*
 * struct rpc_msg_regu_vol_set - regulator voltage set request descriptor
 *
 * @regu_id: Virtual regulator id
 * @is_daul_rail: Specify if this virtual id contains dual rails
 * @dc1: Voltage uint in uv for single rail or the first rail
 * @dc2: Voltage uint in uv for the second rail, ignore it if it's not daul rail
 */
struct rpc_msg_regu_vol_set {
	u16 regu_id;
	u16 is_dual_rail;
	u32 dc1;
	u32 dc2;
	u16 reserved[6];
} __packed __aligned(1);

/*
 * struct rpc_msg_regu_vol_get - regulator voltage get request descriptor
 *
 * @regu_id: Virtual regulator id
 * @is_daul_rail: Specify if this virtual id contains dual rails
 * @dc1: Voltage uint in uv for single rail or the first rail
 * @dc2: Voltage uint in uv for the second rail, ignore it if it's not daul rail
 */
struct rpc_msg_regu_vol_get {
	u16 regu_id;
	u16 is_dual_rail;
	u32 dc1;
	u32 dc2;
	u16 reserved[6];
} __packed __aligned(1);

/*
 * struct rpc_msg_regu_vol_get_ack - regulator voltage get resp descriptor
 *
 * Dedicated for rpc rx message with transform status and regulator voltage.
 *
 * @ack_hdr: RPC message ack descriptor
 * @regu_id: Virtual regulator id
 * @is_daul_rail: Specify if this virtual id contains dual rails
 * @dc1: Voltage uint in uv for single rail or the first rail
 * @dc2: Voltage uint in uv for the second rail, ignore it if it's not daul rail
 */
struct rpc_msg_regu_vol_get_ack {
	struct zhihe_a210_aon_rpc_ack_common ack_hdr;
	u16 regu_id;
	u16 is_dual_rail;
	u32 dc1;
	u32 dc2;
	u16 reserved[6];
} __packed __aligned(1);

/*
 * struct rpc_msg_regu_pwr_set - regulator power status set request descriptor
 *
 * @regu_id: Virtual regulator id
 * @status: Regulator on\off status
 */
struct rpc_msg_regu_pwr_set {
	u16 regu_id;
	u16 status;
	u32 reserved[5];
} __packed __aligned(1);

/*
 * struct rpc_msg_regu_pwr_get - regulator power status get request descriptor
 *
 * @regu_id: Virtual regulator id
 */
struct rpc_msg_regu_pwr_get {
	u16 regu_id;
	u32 reserved[5];
} __packed __aligned(1);

/*
 * struct rpc_msg_regu_pwr_get_ack - regulator power status get resp descriptor
 *
 * @ack_hdr: RPC message ack descriptor
 * @regu_id: Virtual regulator id
 * @status: Regulator on\off status
 */
struct rpc_msg_regu_pwr_get_ack {
	struct zhihe_a210_aon_rpc_ack_common ack_hdr;
	u16 regu_id;
	u16 status;
	u32 reserved[5];
} __packed __aligned(1);

struct zhihe_a210_aon_msg_regulator_ctrl {
	struct zhihe_a210_aon_rpc_msg_hdr hdr;
	union rpc_func_t {
		struct rpc_msg_regu_vol_set regu_vol_set;
		struct rpc_msg_regu_vol_get regu_vol_get;
		struct rpc_msg_regu_pwr_set regu_pwr_set;
		struct rpc_msg_regu_pwr_get regu_pwr_get;
	} __packed __aligned(1) rpc;
} __packed __aligned(1);

enum a210_pm_resource {
	A210_AVDD33_EMMC,       /* sy70209: ldo1  */
	A210_AVDD33_USB2,       /* sy70209: ldo2  */
	A210_DVDD08_AON,        /* sy70209: ldo3  */
	A210_AVDD18_AON,        /* sy70209: ldo4  */
	A210_AVDD18_EMMC_USB2,  /* sy70209: ldo5  */
	A210_AVDD18_EMMC_PERI,  /* sy70209: ldo7  */
	A210_AVDD18_TOP,        /* sy70209: ldo8  */
	A210_AVDD18_PLL,        /* sy70209: ldo9  */
	A210_AVDD18,            /* sy70209: ldo10 */
	A210_DVDD18_DDR_VAA,    /* sy70209: vout1 */
	A210_P3V3,              /* sy70209: vout2 */
	A210_DVDD08_TOP,        /* sy70209: vout3 */
	A210_DVDD06_DDR_VDDQLP, /* sy70209: vout4 */
	A210_DVDD08_DDR,        /* sy70209: vout5 */
	A210_DVDD_CPU,          /* sy70209: vout6 */
	A210_DVDDM_CPU,         /* sy70209: vout7 */
	A210_DVDD_VP,           /* sy70209: vout8 */
	A210_DVDD_NPU_VIP,      /* sy70300: vout1 */
	A210_DVDD_CPU_P,        /* sy70300: vout2 */
	A210_DVDD_GPU,          /* sy70300: vout3 */
	A210_REGU_MAX
};

struct aon_regu_desc {
	const struct regulator_desc *regu_desc;
	unsigned int regu_num;
};

struct aon_regu_info {
	struct device *dev;
	struct zhihe_a210_aon_ipc *ipc_handle;
};

/* dc2 is valid when is_dual_rail is true
 *
 * Dual-rail regulator means that a virtual regulator involves two hardware
 * regulators.
 */
static int aon_set_regulator(struct zhihe_a210_aon_ipc *ipc, u16 regu_id, u32 dc)
{
	struct zhihe_a210_aon_msg_regulator_ctrl msg = { 0 };
	struct zhihe_a210_aon_rpc_ack_common ack_msg = { 0 };
	struct zhihe_a210_aon_rpc_msg_hdr *hdr = &msg.hdr;

	hdr->svc = (u8)ZHIHE_A210_AON_RPC_SVC_PM;
	hdr->func = (u8)ZHIHE_A210_AON_PM_FUNC_SET_RESOURCE_REGULATOR;
	hdr->size = ZHIHE_A210_AON_RPC_MSG_NUM;

	put_unaligned_be16(regu_id, &msg.rpc.regu_vol_set.regu_id);
	put_unaligned_be32(dc, &msg.rpc.regu_vol_set.dc1);

	return zhihe_a210_aon_call_rpc(ipc, &msg, &ack_msg, sizeof(ack_msg), true);
}

/* dc2 is valid when is_dual_rail is true
 *
 * Dual-rail regulator means that a virtual regulator involves two hardware
 * regulators.
 */
static int aon_get_regulator(struct zhihe_a210_aon_ipc *ipc, u16 regu_id, u32 *dc)
{
	struct zhihe_a210_aon_msg_regulator_ctrl msg = { 0 };
	struct rpc_msg_regu_vol_get_ack ack_msg = { 0 };
	struct zhihe_a210_aon_rpc_msg_hdr *hdr = &msg.hdr;
	int ret;

	hdr->svc = (u8)ZHIHE_A210_AON_RPC_SVC_PM;
	hdr->func = (u8)ZHIHE_A210_AON_PM_FUNC_GET_RESOURCE_REGULATOR;
	hdr->size = ZHIHE_A210_AON_RPC_MSG_NUM;

	put_unaligned_be16(regu_id, &msg.rpc.regu_vol_get.regu_id);

	ret = zhihe_a210_aon_call_rpc(ipc, &msg, &ack_msg, sizeof(ack_msg), true);
	if (ret)
		return ret;
	if (dc)
		*dc = get_unaligned_be32(&ack_msg.dc1);

	return 0;
}

static int aon_regu_power_ctrl(struct zhihe_a210_aon_ipc *ipc, u32 regu_id, u16 pwr_on)
{
	struct zhihe_a210_aon_msg_regulator_ctrl msg = { 0 };
	struct zhihe_a210_aon_rpc_ack_common ack_msg = { 0 };
	struct zhihe_a210_aon_rpc_msg_hdr *hdr = &msg.hdr;

	hdr->svc = (u8)ZHIHE_A210_AON_RPC_SVC_PM;
	hdr->func = (u8)ZHIHE_A210_AON_PM_FUNC_PWR_SET;
	hdr->size = ZHIHE_A210_AON_RPC_MSG_NUM;

	put_unaligned_be16(regu_id, &msg.rpc.regu_pwr_set.regu_id);
	put_unaligned_be16(pwr_on, &msg.rpc.regu_pwr_set.status);

	return zhihe_a210_aon_call_rpc(ipc, &msg, &ack_msg, sizeof(ack_msg), true);
}

static int aon_regu_enable(struct regulator_dev *reg)
{
	struct aon_regu_info *info = rdev_get_drvdata(reg);
	u16 regu_id = (u16)rdev_get_id(reg);

	return aon_regu_power_ctrl(info->ipc_handle, regu_id, 1);
}

static int aon_regu_disable(struct regulator_dev *reg)
{
	struct aon_regu_info *info = rdev_get_drvdata(reg);
	u16 regu_id = (u16)rdev_get_id(reg);

	return aon_regu_power_ctrl(info->ipc_handle, regu_id, 0);
}

static int aon_regu_is_enabled(struct regulator_dev *reg)
{
	struct aon_regu_info *info = rdev_get_drvdata(reg);
	struct zhihe_a210_aon_msg_regulator_ctrl msg = { 0 };
	struct rpc_msg_regu_pwr_get_ack ack_msg = { 0 };
	struct zhihe_a210_aon_rpc_msg_hdr *hdr = &msg.hdr;
	u16 regu_id = (u16)rdev_get_id(reg);
	int ret;

	hdr->svc = (u8)ZHIHE_A210_AON_RPC_SVC_PM;
	hdr->func = (u8)ZHIHE_A210_AON_PM_FUNC_PWR_GET;
	hdr->size = ZHIHE_A210_AON_RPC_MSG_NUM;

	put_unaligned_be16(regu_id, &msg.rpc.regu_pwr_get.regu_id);

	ret = zhihe_a210_aon_call_rpc(info->ipc_handle, &msg,
				      &ack_msg, sizeof(ack_msg), true);
	if (ret < 0)
		return ret;

	return !!get_unaligned_be16(&ack_msg.status);
}

static int aon_regu_set_voltage(struct regulator_dev *reg, int minuV,
				int max_uV, unsigned int *selector)
{
	struct aon_regu_info *info = rdev_get_drvdata(reg);
	u16 regu_id = (u16)rdev_get_id(reg);
	int voltage;
	int sel;

	sel = regulator_map_voltage_linear(reg, minuV, max_uV);
	if (sel < 0)
		return sel;
	voltage = regulator_list_voltage_linear(reg, sel);
	if (voltage < 0)
		return voltage;
	*selector = sel;

	return aon_set_regulator(info->ipc_handle, regu_id, voltage);
}

static int aon_regu_get_voltage(struct regulator_dev *reg)
{
	struct aon_regu_info *info = rdev_get_drvdata(reg);
	u16 regu_id = (u16)rdev_get_id(reg);
	int voltage, ret;

	ret = aon_get_regulator(info->ipc_handle, regu_id, &voltage);
	if (ret) {
		dev_err(info->dev, "failed to get regulator %u voltage: %d\n",
			regu_id, ret);
		return ret;
	}

	return voltage;
}

static const struct regulator_ops aon_regu_fixed_ops = {
	.enable =        aon_regu_enable,
	.disable =       aon_regu_disable,
	.is_enabled =    aon_regu_is_enabled,
	.get_voltage =   aon_regu_get_voltage,
};

static const struct regulator_ops aon_regu_variable_ops = {
	.enable =        aon_regu_enable,
	.disable =       aon_regu_disable,
	.is_enabled =    aon_regu_is_enabled,
	.list_voltage =  regulator_list_voltage_linear,
	.set_voltage =   aon_regu_set_voltage,
	.get_voltage =   aon_regu_get_voltage,
};

static const struct regulator_desc zhihe_a210_aon_regu_desc[] = {
	/* Common regulators do not need dynamic voltage adjustment. */
	{
		.id = A210_AVDD33_EMMC,
		.name = "avdd33_emmc",
		.of_match = "avdd33_emmc",
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_AVDD33_USB2,
		.name = "avdd33_usb2",
		.of_match = "avdd33_usb2",
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_DVDD08_AON,
		.name = "dvdd08_aon",
		.of_match = "dvdd08_aon",
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_AVDD18_AON,
		.name = "avdd18_aon",
		.of_match = "avdd18_aon",
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_AVDD18_EMMC_USB2,
		.name = "avdd18_emmc_usb2",
		.of_match = "avdd18_emmc_usb2",
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_AVDD18_EMMC_PERI,
		.name = "avdd18_emmc_peri",
		.of_match = "avdd18_emmc_peri",
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_AVDD18_TOP,
		.name = "avdd18_top",
		.of_match = "avdd18_top",
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_AVDD18_PLL,
		.name = "avdd18_pll",
		.of_match = "avdd18_pll",
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_AVDD18,
		.name = "avdd18",
		.of_match = "avdd18",
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_DVDD18_DDR_VAA,
		.name = "dvdd18_ddr_vaa",
		.of_match = "dvdd18_ddr_vaa",
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_P3V3,
		.name = "p3v3",
		.of_match = "p3v3",
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_DVDD08_TOP,
		.name = "dvdd08_top",
		.of_match = "dvdd08_top",
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_DVDD06_DDR_VDDQLP,
		.name = "dvdd06_ddr_vddqlp",
		.of_match = "dvdd06_ddr_vddqlp",
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_DVDD08_DDR,
		.name = "dvdd08_ddr",
		.of_match = "dvdd08_ddr",
		.min_uV = 750000,
		.uV_step = 5000,
		.n_voltages = (800000 - 750000) / 5000 + 1,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_DVDD_CPU,
		.name = "dvdd_cpu",
		.of_match = "dvdd_cpu",
		.min_uV = 600000,
		.uV_step = 12500,
		.n_voltages = (1000000 - 600000) / 12500 + 1,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_DVDDM_CPU,
		.name = "dvddm_cpu",
		.of_match = "dvddm_cpu",
		.min_uV = 800000,
		.uV_step = 12500,
		.n_voltages = (1000000 - 800000) / 12500 + 1,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_DVDD_VP,
		.name = "dvdd_vp",
		.of_match = "dvdd_vp",
		.min_uV = 750000,
		.uV_step = 12500,
		.n_voltages = (800000 - 750000) / 12500 + 1,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_DVDD_NPU_VIP,
		.name = "dvdd_npu_vip",
		.of_match = "dvdd_npu_vip",
		.min_uV = 750000,
		.uV_step = 5000,
		.n_voltages = (1000000 - 750000) / 5000 + 1,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_DVDD_CPU_P,
		.name = "dvdd_cpu_p",
		.of_match = "dvdd_cpu_p",
		.min_uV = 800000,
		.uV_step = 5000,
		.n_voltages = (1000000 - 800000) / 5000 + 1,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = A210_DVDD_GPU,
		.name = "dvdd_gpu",
		.of_match = "dvdd_gpu",
		.min_uV = 750000,
		.uV_step = 5000,
		.n_voltages = (800000 - 750000) / 5000 + 1,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

static const struct aon_regu_desc zhihe_a210_aon_regus = {
	.regu_desc = zhihe_a210_aon_regu_desc,
	.regu_num  = ARRAY_SIZE(zhihe_a210_aon_regu_desc),
};

static int zhihe_match_regulator_id(const struct aon_regu_desc *regus_set, u32 id)
{
	int index;

	for (index = 0; index < regus_set->regu_num; index++) {
		if (regus_set->regu_desc[index].id == id)
			return index;
	}

	return -EINVAL;
}

static int zhihe_a210_aon_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct aon_regu_desc *regus_set;
	struct aon_regu_info *info;
	struct regulator_config config = {};
	struct regulator_desc *desc;
	int ret;

	regus_set = device_get_match_data(dev);
	if (!regus_set)
		return -ENODEV;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->dev = dev;
	info->ipc_handle = dev_get_drvdata(dev->parent);
	if (!info->ipc_handle)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "AON RPC transport is not ready\n");

	config.dev = dev;
	config.driver_data = info;

	for_each_available_child_of_node_scoped(dev->of_node, child) {
		const struct regulator_desc *template;
		struct regulator_dev *rdev;
		u32 regulator_id;
		int index;

		ret = of_property_read_u32(child, "zhihe,regulator-id", &regulator_id);
		if (ret)
			return dev_err_probe(dev, ret,
					     "missing regulator ID for %pOFn\n",
					     child);

		index = zhihe_match_regulator_id(regus_set, regulator_id);
		if (index < 0)
			return dev_err_probe(dev, index,
					     "no regulator matches ID %u\n",
					     regulator_id);

		template = &regus_set->regu_desc[index];
		desc = devm_kmemdup(dev, template, sizeof(*desc), GFP_KERNEL);
		if (!desc)
			return -ENOMEM;
		desc->ops = desc->n_voltages ? &aon_regu_variable_ops :
						&aon_regu_fixed_ops;

		config.of_node = child;
		rdev = devm_regulator_register(dev, desc, &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     desc->name);
	}

	platform_set_drvdata(pdev, info);
	return 0;
}

static const struct of_device_id zhihe_pmic_dev_id[] = {
	{ .compatible = "zhihe,a210-aon-regulator", .data = &zhihe_a210_aon_regus},
	{},
};
MODULE_DEVICE_TABLE(of, zhihe_pmic_dev_id);

static struct platform_driver zhihe_a210_aon_regulator_driver = {
	.driver = {
		.name = "zhihe-a210-aon-regulator",
		.of_match_table = zhihe_pmic_dev_id,
	},
	.probe = zhihe_a210_aon_regulator_probe,
};
module_platform_driver(zhihe_a210_aon_regulator_driver);

MODULE_AUTHOR("Hongkun Xu <xuhongkun@zhcomputing.com>");
MODULE_DESCRIPTION("ZhiHe A210 AON regulator driver");
MODULE_LICENSE("GPL");
