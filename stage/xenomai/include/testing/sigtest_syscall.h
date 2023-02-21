#ifndef SIGTEST_SYSCALL_H
#define SIGTEST_SYSCALL_H

#include <asm/xenomai/syscall.h>

#define SIGTEST_SKIN_MAGIC 0x53494754

#define __NR_sigtest_queue 0	/* sigtest_queue(int *, size_t) */
#define __NR_sigtest_wait_pri 1	/* sigtest_wait_pri(void) */
#define __NR_sigtest_wait_sec 2 /* sigtest_wait_sec(void) */

struct sigtest_siginfo {
	int sig_nr;
};

#endif /* SIGTEST_SYSCALL_H */
