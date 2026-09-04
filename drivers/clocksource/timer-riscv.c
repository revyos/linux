// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2012 Regents of the University of California
 * Copyright (C) 2017 SiFive
 *
 * All RISC-V systems have a timer attached to every hart.  These timers can
 * either be read from the "time" and "timeh" CSRs, and can use the SBI to
 * setup events, or directly accessed using MMIO registers.
 */

#define pr_fmt(fmt) "riscv-timer: " fmt

#include <linux/acpi.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/sched_clock.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/limits.h>
#include <clocksource/timer-riscv.h>
#include <asm/smp.h>
#include <asm/cpufeature.h>
#include <asm/sbi.h>
#include <asm/timex.h>

static DEFINE_STATIC_KEY_FALSE(riscv_sstc_available);
static bool riscv_timer_cannot_wake_cpu;

#define A210_CLINT_STIMECMP_OFFSET	0xd000

static u8 __iomem *a210_stimecmp_base;
static unsigned int a210_stimecmp_count;

static bool a210_stimecmp_set_next(u64 next_tval)
{
	unsigned long hartid = cpuid_to_hartid_map(raw_smp_processor_id());
	void __iomem *stimecmp;

	if (!a210_stimecmp_base || hartid >= a210_stimecmp_count)
		return false;

	stimecmp = a210_stimecmp_base + hartid * sizeof(next_tval);
	writel_relaxed(U32_MAX, stimecmp);
	writel_relaxed(upper_32_bits(next_tval), stimecmp + sizeof(u32));
	writel_relaxed(lower_32_bits(next_tval), stimecmp);

	return true;
}

static void __init a210_stimecmp_init(void)
{
	struct device_node *np;
	struct resource res;
	resource_size_t size;

	if (!of_machine_is_compatible("zhihe,a210"))
		return;

	np = of_find_compatible_node(NULL, NULL, "thead,c900-clint");
	if (!np)
		return;

	if (of_address_to_resource(np, 0, &res))
		goto out_put;

	size = resource_size(&res);
	if (size <= A210_CLINT_STIMECMP_OFFSET)
		goto out_put;

	size -= A210_CLINT_STIMECMP_OFFSET;
	a210_stimecmp_base = ioremap(res.start + A210_CLINT_STIMECMP_OFFSET, size);
	if (!a210_stimecmp_base)
		goto out_put;

	a210_stimecmp_count = size / sizeof(u64);
	pr_info("using A210 S-mode timer compare registers\n");

out_put:
	of_node_put(np);
}

static void riscv_clock_event_stop(void)
{
	if (static_branch_likely(&riscv_sstc_available)) {
		csr_write(CSR_STIMECMP, ULONG_MAX);
		if (IS_ENABLED(CONFIG_32BIT))
			csr_write(CSR_STIMECMPH, ULONG_MAX);
	} else if (!a210_stimecmp_set_next(U64_MAX)) {
		sbi_set_timer(U64_MAX);
	}
}

static int riscv_clock_next_event(unsigned long delta,
		struct clock_event_device *ce)
{
	u64 next_tval = get_cycles64() + delta;

	if (static_branch_likely(&riscv_sstc_available)) {
#if defined(CONFIG_32BIT)
		csr_write(CSR_STIMECMP, ULONG_MAX);
		csr_write(CSR_STIMECMPH, next_tval >> 32);
		csr_write(CSR_STIMECMP, next_tval & 0xFFFFFFFF);
#else
		csr_write(CSR_STIMECMP, next_tval);
#endif
	} else if (!a210_stimecmp_set_next(next_tval))
		sbi_set_timer(next_tval);

	return 0;
}

static int riscv_clock_shutdown(struct clock_event_device *evt)
{
	riscv_clock_event_stop();
	return 0;
}

static unsigned int riscv_clock_event_irq;
static DEFINE_PER_CPU(struct clock_event_device, riscv_clock_event) = {
	.name			= "riscv_timer_clockevent",
	.features		= CLOCK_EVT_FEAT_ONESHOT,
	.rating			= 100,
	.set_next_event		= riscv_clock_next_event,
	.set_state_shutdown	= riscv_clock_shutdown,
};

/*
 * It is guaranteed that all the timers across all the harts are synchronized
 * within one tick of each other, so while this could technically go
 * backwards when hopping between CPUs, practically it won't happen.
 */
static unsigned long long riscv_clocksource_rdtime(struct clocksource *cs)
{
	return get_cycles64();
}

static u64 notrace riscv_sched_clock(void)
{
	return get_cycles64();
}

static struct clocksource riscv_clocksource = {
	.name		= "riscv_clocksource",
	.rating		= 400,
	.mask		= CLOCKSOURCE_MASK(64),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
	.read		= riscv_clocksource_rdtime,
	.vdso_clock_mode = VDSO_CLOCKMODE_ARCHTIMER,
};

static int riscv_timer_starting_cpu(unsigned int cpu)
{
	struct clock_event_device *ce = per_cpu_ptr(&riscv_clock_event, cpu);

	/* Clear timer interrupt */
	riscv_clock_event_stop();

	ce->cpumask = cpumask_of(cpu);
	ce->irq = riscv_clock_event_irq;
	if (riscv_timer_cannot_wake_cpu)
		ce->features |= CLOCK_EVT_FEAT_C3STOP;
	if (static_branch_likely(&riscv_sstc_available))
		ce->rating = 450;
	clockevents_config_and_register(ce, riscv_timebase, 100, ULONG_MAX);

	enable_percpu_irq(riscv_clock_event_irq,
			  irq_get_trigger_type(riscv_clock_event_irq));
	return 0;
}

static int riscv_timer_dying_cpu(unsigned int cpu)
{
	/*
	 * Stop the timer when the cpu is going to be offline otherwise
	 * the timer interrupt may be pending while performing power-down.
	 */
	riscv_clock_event_stop();
	disable_percpu_irq(riscv_clock_event_irq);

	return 0;
}

void riscv_cs_get_mult_shift(u32 *mult, u32 *shift)
{
	*mult = riscv_clocksource.mult;
	*shift = riscv_clocksource.shift;
}
EXPORT_SYMBOL_GPL(riscv_cs_get_mult_shift);

/* called directly from the low-level interrupt handler */
static irqreturn_t riscv_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evdev = this_cpu_ptr(&riscv_clock_event);

	riscv_clock_event_stop();
	evdev->event_handler(evdev);

	return IRQ_HANDLED;
}

static int __init riscv_timer_init_common(void)
{
	int error;
	struct irq_domain *domain;
	struct fwnode_handle *intc_fwnode = riscv_get_intc_hwnode();

	domain = irq_find_matching_fwnode(intc_fwnode, DOMAIN_BUS_ANY);
	if (!domain) {
		pr_err("Failed to find irq_domain for INTC node [%pfwP]\n",
		       intc_fwnode);
		return -ENODEV;
	}

	riscv_clock_event_irq = irq_create_mapping(domain, RV_IRQ_TIMER);
	if (!riscv_clock_event_irq) {
		pr_err("Failed to map timer interrupt for node [%pfwP]\n", intc_fwnode);
		return -ENODEV;
	}

	error = clocksource_register_hz(&riscv_clocksource, riscv_timebase);
	if (error) {
		pr_err("RISCV timer registration failed [%d]\n", error);
		return error;
	}

	sched_clock_register(riscv_sched_clock, 64, riscv_timebase);

	error = request_percpu_irq(riscv_clock_event_irq,
				    riscv_timer_interrupt,
				    "riscv-timer", &riscv_clock_event);
	if (error) {
		pr_err("registering percpu irq failed [%d]\n", error);
		return error;
	}

	if (riscv_isa_extension_available(NULL, SSTC)) {
		pr_info("Timer interrupt in S-mode is available via sstc extension\n");
		static_branch_enable(&riscv_sstc_available);
	}

	error = cpuhp_setup_state(CPUHP_AP_RISCV_TIMER_STARTING,
			 "clockevents/riscv/timer:starting",
			 riscv_timer_starting_cpu, riscv_timer_dying_cpu);
	if (error)
		pr_err("cpu hp setup state failed for RISCV timer [%d]\n",
		       error);

	return error;
}

static int __init riscv_timer_init_dt(struct device_node *n)
{
	int cpuid, error;
	unsigned long hartid;
	struct device_node *child;

	error = riscv_of_processor_hartid(n, &hartid);
	if (error < 0) {
		pr_warn("Invalid hartid for node [%pOF] error = [%lu]\n",
			n, hartid);
		return error;
	}

	cpuid = riscv_hartid_to_cpuid(hartid);
	if (cpuid < 0) {
		pr_warn("Invalid cpuid for hartid [%lu]\n", hartid);
		return cpuid;
	}

	if (cpuid != smp_processor_id())
		return 0;

	a210_stimecmp_init();

	child = of_find_compatible_node(NULL, NULL, "riscv,timer");
	if (child) {
		riscv_timer_cannot_wake_cpu = of_property_read_bool(child,
					"riscv,timer-cannot-wake-cpu");
		of_node_put(child);
	}

	return riscv_timer_init_common();
}

TIMER_OF_DECLARE(riscv_timer, "riscv", riscv_timer_init_dt);

#ifdef CONFIG_ACPI
static int __init riscv_timer_acpi_init(struct acpi_table_header *table)
{
	struct acpi_table_rhct *rhct = (struct acpi_table_rhct *)table;

	riscv_timer_cannot_wake_cpu = rhct->flags & ACPI_RHCT_TIMER_CANNOT_WAKEUP_CPU;

	return riscv_timer_init_common();
}

TIMER_ACPI_DECLARE(aclint_mtimer, ACPI_SIG_RHCT, riscv_timer_acpi_init);

#endif
