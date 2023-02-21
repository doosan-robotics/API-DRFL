#ifndef _XENO_ASM_SH_BIND_H
#define _XENO_ASM_SH_BIND_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <limits.h>

#include <asm-generic/xenomai/bind.h>

struct xnarch_tsc_area;

volatile struct xnarch_tsc_area *xeno_sh_tsc = NULL;

volatile unsigned long *xeno_sh_tcnt = NULL;

static volatile void *__xeno_kmem_map(unsigned long pa, unsigned int pagesz)
{
	void *p;
	int fd;

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd == -1) {
		perror("Xenomai init: open(/dev/mem)");
		exit(EXIT_FAILURE);
	}

	p = mmap(NULL, pagesz, PROT_READ | PROT_WRITE, MAP_SHARED, fd, pa & ~(pagesz - 1));
	if (p == MAP_FAILED) {
		perror("Xenomai init: mmap(/dev/mem)");
		exit(EXIT_FAILURE);
	}
	close(fd);

	return (volatile void *)(p + (pa & (pagesz - 1)));
}

static inline void xeno_sh_features_check(struct xnfeatinfo *finfo)
{
	unsigned int pagesz = sysconf(_SC_PAGESIZE);
	xeno_sh_tsc = __xeno_kmem_map(finfo->feat_arch.hrclock_membase, pagesz);
	xeno_sh_tcnt = __xeno_kmem_map(xeno_sh_tsc->counter_pa, pagesz);
}

#define xeno_arch_features_check(finfo) xeno_sh_features_check(finfo)

#endif /* _XENO_ASM_SH_BIND_H */
