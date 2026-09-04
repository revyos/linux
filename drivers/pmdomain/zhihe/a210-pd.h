/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2025 Zhihe Computing Limited.
 */

#ifndef __A210_PD_H
#define __A210_PD_H

#define A210_PD_GPU			0
#define A210_PD_NPU_WRAPPER		1
#define A210_PD_NPU_IP			2
#define A210_PD_D2D			3
#define A210_PD_PCIE0			4
#define A210_PD_PCIE1			5
#define A210_PD_SATA			6
#define A210_PD_USB			7
#define A210_PD_VI_WRAP			8
#define A210_PD_VI_ISP			9
#define A210_PD_VO			10
#define A210_PD_VP_WRAP			11
#define A210_PD_VENC			12
#define A210_PD_VDEC			13
#define A210_PD_TOP			14
#define A210_PD_PERI0			15
#define A210_PD_PERI1			16
#define A210_PD_PERI2			17
#define A210_PD_PERI3			18
#define A210_PD_CAN			19
#define A210_PD_SPI			20
#define A210_PD_QSPI			21
#define A210_PD_UART			22
#define A210_PD_PWM			23
#define A210_PD_EMMC			24
#define A210_PD_SDIO			25
#define A210_PD_DMAC			26
#define A210_POWER_DOMAINS_MAX		27
#define CCU_MODE_GATING			0
#define CCU_MODE_SCALING		1

#define BPC_HW_MODEL 0
#define PCU_REG_TRIGGER 1

/* represent power domains info at soc level */
struct a210_pd_soc {
	struct device *dev;
	struct a210_pm_domain *domains[A210_POWER_DOMAINS_MAX];
	unsigned int num_domains;
	struct regulator *regulators[A210_POWER_DOMAINS_MAX];
};

struct a210_ccu {
	u32 delay;
	u32 step;
	u32 ratio;
	void __iomem *base;
};

struct a210_ccu_grp {
	u32 mode;
	unsigned int num_ccus;
	struct list_head list;
	struct a210_ccu ccu[];
};

/* represent a single power domain */
struct a210_pm_domain {
	struct generic_pm_domain pd;
	u16 index;
	struct a210_pd_soc *soc;
	struct device_node *np;
	struct reset_control *reset;
	struct clk_bulk_data *clks;
	unsigned int num_clks;
	void __iomem *pca_base;
	void __iomem *bpc_base;
	void __iomem *pcu_base;
	struct list_head ccu_grp;
};

enum power_mode {
	OFF = 0x0,
	ON = 0x1f,
};

#endif
