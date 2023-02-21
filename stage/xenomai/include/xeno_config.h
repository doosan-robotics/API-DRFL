/* src/include/xeno_config.h.  Generated from xeno_config.h.in by configure.  */
/* src/include/xeno_config.h.in.  Generated from configure.in by autoheader.  */

/* config */
#define CONFIG_MMU 1

/* config */
#define CONFIG_SMP 1

/* config */
#define CONFIG_X86_TSC 1

/* config */
/* #undef CONFIG_XENO_ARM_SA1100 */

/* config */
/* #undef CONFIG_XENO_ARM_TSC_TYPE */

/* config */
/* #undef CONFIG_XENO_ATOMIC_BUILTINS */

/* Build system alias */
#define CONFIG_XENO_BUILD_STRING "i686-pc-linux-gnu"

/* Compiler */
#define CONFIG_XENO_COMPILER "gcc version 4.8.5 (Ubuntu 4.8.5-4ubuntu6) "

/* config */
/* #undef CONFIG_XENO_CPU_XSC3 */

/* config */
#define CONFIG_XENO_DEFAULT_PERIOD 100000

/* config */
#define CONFIG_XENO_FASTSYNCH 1

/* configure */
#define CONFIG_XENO_FORTIFY 1

/* config */
#define CONFIG_XENO_HAVE_MQUEUE_H 

/* config */
#define CONFIG_XENO_HAVE_PTHREAD_CONDATTR_SETCLOCK 

/* config */
#define CONFIG_XENO_HAVE_PTHREAD_MUTEXATTR_SETPROTOCOL 

/* Host system alias */
#define CONFIG_XENO_HOST_STRING "i686-pc-linux-gnu"

/* config */
#define CONFIG_XENO_REVISION_LEVEL 5

/* config */
#define CONFIG_XENO_VERSION_MAJOR 2

/* config */
#define CONFIG_XENO_VERSION_MINOR 6

/* config */
#define CONFIG_XENO_X86_SEP 1

#ifdef __IN_XENO__

/* Define to 1 if you have the <dlfcn.h> header file. */
#define HAVE_DLFCN_H 1

/* Define to 1 if you have the `fork' function. */
#define HAVE_FORK 1

/* Define to 1 if you have the `ftruncate64' function. */
#define HAVE_FTRUNCATE64 1

/* Define to 1 if you have the <inttypes.h> header file. */
#define HAVE_INTTYPES_H 1

/* Define to 1 if you have the <memory.h> header file. */
#define HAVE_MEMORY_H 1

/* Define to 1 if you have the `mmap64' function. */
#define HAVE_MMAP64 1

/* Define to 1 if you have the <mqueue.h> header file. */
#define HAVE_MQUEUE_H 1

/* config */
/* #undef HAVE_OLD_SETAFFINITY */

/* Define to 1 if you have the `open64' function. */
#define HAVE_OPEN64 1

/* Define to 1 if you have the `pthread_condattr_setclock' function. */
#define HAVE_PTHREAD_CONDATTR_SETCLOCK 1

/* Define to 1 if you have the `pthread_mutexattr_setprotocol' function. */
#define HAVE_PTHREAD_MUTEXATTR_SETPROTOCOL 1

/* Define to 1 if you have the `pthread_spin_lock' function. */
#define HAVE_PTHREAD_SPIN_LOCK 1

/* config */
#define HAVE_RECENT_SETAFFINITY 1

/* Define to 1 if you have the `shm_open' function. */
#define HAVE_SHM_OPEN 1

/* Define to 1 if you have the `shm_unlink' function. */
#define HAVE_SHM_UNLINK 1

/* Define to 1 if you have the <stdint.h> header file. */
#define HAVE_STDINT_H 1

/* Define to 1 if you have the <stdlib.h> header file. */
#define HAVE_STDLIB_H 1

/* Define to 1 if you have the <strings.h> header file. */
#define HAVE_STRINGS_H 1

/* Define to 1 if you have the <string.h> header file. */
#define HAVE_STRING_H 1

/* Define to 1 if you have the <sys/stat.h> header file. */
#define HAVE_SYS_STAT_H 1

/* Define to 1 if you have the <sys/types.h> header file. */
#define HAVE_SYS_TYPES_H 1

/* Define to 1 if you have the <unistd.h> header file. */
#define HAVE_UNISTD_H 1

/* config */
/* #undef HAVE___THREAD */

/* Define to the sub-directory in which libtool stores uninstalled libraries.
   */
#define LT_OBJDIR ".libs/"

/* Name of package */
#define PACKAGE "xenomai"

/* Define to the address where bug reports for this package should be sent. */
#define PACKAGE_BUGREPORT "xenomai@xenomai.org"

/* Define to the full name of this package. */
#define PACKAGE_NAME "Xenomai"

/* Define to the full name and version of this package. */
#define PACKAGE_STRING "Xenomai 2.6.5"

/* Define to the one symbol short name of this package. */
#define PACKAGE_TARNAME "xenomai"

/* Define to the home page for this package. */
#define PACKAGE_URL ""

/* Define to the version of this package. */
#define PACKAGE_VERSION "2.6.5"

/* Define to 1 if you have the ANSI C header files. */
#define STDC_HEADERS 1

/* Version number of package */
#define VERSION "2.6.5"

/* Define to 1 if `lex' declares `yytext' as a `char *' by default, not a
   `char[]'. */
/* #undef YYTEXT_POINTER */

#endif /* __IN_XENO__ */
