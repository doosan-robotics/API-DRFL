#ifndef _XENO_ASM_NIOS2_FPTEST_H
#define _XENO_ASM_NIOS2_FPTEST_H

#ifdef __KERNEL__
#include <linux/module.h>

static inline int fp_kernel_supported(void)
{
	return 0;
}

static inline int fp_linux_begin(void)
{
	return -ENOSYS;
}

static inline void fp_linux_end(void)
{
}

#else /* !__KERNEL__ */
#include <stdio.h>
#define printk printf
#endif /* !__KERNEL__ */

static inline void fp_features_init(void)
{
}

static inline void fp_regs_set(unsigned val)
{
}

static inline unsigned fp_regs_check(unsigned val)
{
    return val;
}

#endif /* _XENO_ASM_NIOS2_FPTEST_H */
