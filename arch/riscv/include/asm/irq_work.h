/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_RISCV_IRQ_WORK_H
#define _ASM_RISCV_IRQ_WORK_H

static inline bool arch_irq_work_has_interrupt(void)
{
	/*
	 * On single-core systems built with CONFIG_SMP=y, avoid sending a
	 * self-IPI to raise irq_work: old firmware (OpenSBI < v1.4) does not
	 * deliver self-IPIs correctly when the CLINT is absent from the M-mode
	 * device tree.  When this returns false, irq_work_tick() processes the
	 * raised_list on every timer tick as a safe fallback, matching the
	 * behaviour of !SMP kernels.
	 */
	return IS_ENABLED(CONFIG_SMP) && num_possible_cpus() > 1;
}

#endif /* _ASM_RISCV_IRQ_WORK_H */
