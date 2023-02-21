#pragma once
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <math.h>
#include <string.h>

#if defined (OS_LINUX)
#include <unistd.h>
#ifdef __XENO__
#include <rtdk.h>
#include <native/timer.h>
#endif
#endif

#define _WINSOCKAPI_ //prevent socket redefinition error
//#include "vkcode.h"  // keyboard event handler

#include "TaskUtil.h"
//#include "Encrypt/PufEncrypt.h"
//#include "configure.h"

#define ANALOGIO_DEFAULT_VOLTAGE
#define _FUNC_SYSTEM_RESTART

#define _FUNC_REMOTECONTROL_NEW_CONCEPT
#define _FUNC_SAFETY_PRS_RESUME
#define _FUNC_3POSSWITHC_NEW_CONCEPT
#define _FUNC_DIRECTTEACH_NEW_CONCEPT
#define _FUNC_SAFETY_PRS_RESET
#define _FUNC_POPUPCOMMAND_NEW_CONCEPT
#define _FUNC_SAEFTY_HOLDTORN_CONCEPT
#define _FUNC_PROTECTIVE_SAFE_OFF_CONCEPT
#define _FUNC_SAFETYIO_EXTENSION
#define _FUNC_PROGRAM_CONTROL_LIMIT // program pasue and stop in not allowd in teaching state
#define _FUNC_SAFETY_INTERLOCK_RESET


#if defined (_FUNC_A_SERIES)
#define FTS_ROBOT_MODEL_AXXXXS_MARK
//#define _FUNC_DYNAMIC_TOUCH_ACTIVATE    //Dynamically activate touch device
#endif

#define STACK_SIZE_INCREASE
#if !defined(STACK_SIZE_INCREASE)
const int TASK_STACK_SIZE = (1024*1024); //0x20000; //0x10000
#define TASK_IDLE_STACK_SIZE TASK_STACK_SIZE
#define TASK_PYTH_STACK_SIZE TASK_STACK_SIZE
#define TASK_SYS_STACK_SIZE  TASK_STACK_SIZE
#define TASK_DEV_STACK_SIZE  TASK_STACK_SIZE
#define TASK_CTL_STACK_SIZE  TASK_STACK_SIZE
#define TASK_USR_STACK_SIZE  TASK_STACK_SIZE
#else

const int TASK_STACK_SIZE_1K = (128 * 1024);
const int TASK_STACK_SIZE_2K = (256 * 1024);
const int TASK_STACK_SIZE_5K = (512 * 1024);
const int TASK_STACK_SIZE_1M = (1024 * 1024);
const int TASK_STACK_SIZE_2M = (2 * 1024 * 1024);
const int TASK_STACK_SIZE_4M = (4 * 1024 * 1024);
const int TASK_STACK_SIZE_8M = (8 * 1024 * 1024);

/* 2x increase for model integration */
#if 1
#define TASK_STACK_SIZE      TASK_STACK_SIZE_2M
#define TASK_IDLE_STACK_SIZE TASK_STACK_SIZE_8M
#define TASK_PYTH_STACK_SIZE TASK_STACK_SIZE_8M
#define TASK_SYS_STACK_SIZE  TASK_STACK_SIZE_2M
#define TASK_DEV_STACK_SIZE  TASK_STACK_SIZE_2M
#define TASK_CTL_STACK_SIZE  TASK_STACK_SIZE_8M
#define TASK_USR_STACK_SIZE  TASK_STACK_SIZE_2M
#else
#define TASK_STACK_SIZE      TASK_STACK_SIZE_1M
#define TASK_IDLE_STACK_SIZE TASK_STACK_SIZE_4M
#define TASK_PYTH_STACK_SIZE TASK_STACK_SIZE_4M
#define TASK_SYS_STACK_SIZE  TASK_STACK_SIZE_1M
#define TASK_DEV_STACK_SIZE  TASK_STACK_SIZE_1M
#define TASK_CTL_STACK_SIZE  TASK_STACK_SIZE_4M
#define TASK_USR_STACK_SIZE  TASK_STACK_SIZE_1M
#endif
#endif

//
//  Time value constants
//
const double TIME_NANOSECOND  = 1e9;
const double TIME_MICROSECOND = 1e6;
const double TIME_MILISECOND  = 1e3;


enum
{
    CPU_0 = 0,
    CPU_1,
    CPU_2,
    CPU_3
};

enum {
#ifdef CPU0_NONE_TASK
    SYS_CPU = CPU_3,
    DEV_CPU = CPU_1,
    CTL_CPU = CPU_2,
    USR_CPU = CPU_3,
    LOG_CPU = CPU_3,
    USP_CPU = CPU_0,
    PYT_CPU = USP_CPP
    SCB_CPU = USP_CPP
#else
#ifdef CPU3_NONE_TASK
    SYS_CPU = CPU_2,
    DEV_CPU = CPU_1,
    CTL_CPU = CPU_0,
    USR_CPU = CPU_2,
    LOG_CPU = CPU_2,
    USP_CPU = CPU_3,
    PYT_CPU = USP_CPU,
    SCB_CPU = USP_CPU
#else
    SYS_CPU = CPU_0,
    DEV_CPU = CPU_1,
    CTL_CPU = CPU_2,
    USR_CPU = CPU_3,
    LOG_CPU = CPU_3,
    USP_CPP = CPU_0,
    PYT_CPU = USP_CPU,
    SCB_CPU = USP_CPU
#endif
#endif
};

//
//  Priority value constants
//

#if defined (OS_LINUX)
enum
{
    PRIORITY_CRITICAL     = 99,
    PRIORITY_HIGHEST      = 97,
    PRIORITY_HIGHHIGH     = 95,
    PRIORITY_HIGH         = 90,
    PRIORITY_ABOVE_NORMAL = 87,
    PRIORITY_NORMAL       = 85,
    PRIORITY_BELOW_NORMAL = 83,
    PRIORITY_LOW          = 80,
    PRIORITY_LOWLOW       = 78,
    PRIORITY_LOWEST       = 75
};
#else
enum
{
    PRIORITY_CRITICAL     = THREAD_PRIORITY_TIME_CRITICAL,
    PRIORITY_HIGHEST      = THREAD_PRIORITY_HIGHEST,
    PRIORITY_HIGHHIGH     = THREAD_PRIORITY_ABOVE_NORMAL,
    PRIORITY_HIGH         = THREAD_PRIORITY_NORMAL,
    PRIORITY_ABOVE_NORMAL = THREAD_PRIORITY_BELOW_NORMAL,
    PRIORITY_NORMAL   = THREAD_PRIORITY_BELOW_NORMAL,
    PRIORITY_BELOW_NORMAL = THREAD_PRIORITY_BELOW_NORMAL,
    PRIORITY_LOW          = THREAD_PRIORITY_LOWEST,
    PRIORITY_LOWLOW       = THREAD_PRIORITY_LOWEST,
    PRIORITY_LOWEST       = THREAD_PRIORITY_IDLE,
};
#endif

#define PRIORITY_SOCKET             PRIORITY_LOW

//
// Scheduling policies of Linux threads
//
/*
  SCHED_OTHER의 경우 명시적인 Schedule을 보장 하지 않음에 유의!
  보장 받기 위해서는 SCHED_FIFO나 SCHED_RR로 설정 하고, high Priority(현재=75)를 할당 해야 한다.
*/
#define SCHED_OTHER     0
#define SCHED_FIFO      1 //* 현재 디폴트
#define SCHED_RR        2
#define SCHED_BATCH     3

//
// function definitions
//

#ifdef _DEBUG
#define UNUSED(x) assert(FALSE);
#else
#define UNUSED(x) void();
#endif

#ifndef SAFE_DELETE_NEW
#define SAFE_DELETE_NEW(p)  {   if (p) {    delete p;   p = NULL;   }   }
#endif

#ifndef SAFE_DELETE_ARRAY
#define SAFE_DELETE_ARRAY(p)  { if (p) {    delete[] p; p = NULL;   }   }
#endif


#ifndef MAKELONG
#define MAKELONG(a, b)  ((long)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned long)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define LOWORD(l)       ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define HIWORD(l)       ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#endif

#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define CHK_XENO_CALL()                                                  \
{                                                                        \
    do { if(rt_task_self() == 0) {                                       \
    rt_printf("RTT: XENO NOT INITIALISED IN THIS THREAD pid=%d,\n\
                BUT TRIES TO INVOKE XENO FUNCTION >>%s<< ANYWAY\n",      \
                getpid(), __FUNCTION__ );                                \
                assert( rt_task_self() != 0 );                           \
    }                                                                    \
    } while(0);                                                          \
}

//
// data type definitions
//
