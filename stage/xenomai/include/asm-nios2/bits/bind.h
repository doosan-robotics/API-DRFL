#ifndef _XENO_ASM_NIOS2_BIND_H
#define _XENO_ASM_NIOS2_BIND_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <limits.h>

#include <asm-generic/xenomai/bind.h>

volatile void *xeno_nios2_hrclock = NULL;

static inline void xeno_nios2_features_check(struct xnfeatinfo *finfo)
{
	unsigned long pa = finfo->feat_arch.hrclock_membase;
	unsigned int pagesz;
	void *p;
	int fd;

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd == -1) {
		perror("Xenomai init: open(/dev/mem)");
		exit(EXIT_FAILURE);
	}

	pagesz = sysconf(_SC_PAGESIZE);
	p = mmap(NULL, pagesz, PROT_READ | PROT_WRITE, MAP_SHARED, fd, pa & ~(pagesz - 1));
	if (p == MAP_FAILED) {
		perror("Xenomai init: mmap(/dev/mem)");
		exit(EXIT_FAILURE);
	}
	close(fd);

	xeno_nios2_hrclock = (volatile void *)(p + (pa & (pagesz - 1)));
}

#define xeno_arch_features_check(finfo) xeno_nios2_features_check(finfo)

#endif /* _XENO_ASM_NIOS2_BIND_H */
