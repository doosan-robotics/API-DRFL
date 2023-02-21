#pragma once
#include <stdio.h>
#include <stdarg.h>
//#include "configure.h"
#if defined (__XENO__)
#include <native/task.h>
#endif

#include <unistd.h>


namespace DRAFramework {

    class CTaskUtil
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////
        CTaskUtil();
        virtual~CTaskUtil();

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////
#if defined(NEW_EVENT_LOG)
        static void DebugString(unsigned int nMajorCategory, unsigned int nMinorCategory, const char *format, ...);
        static void TraceString(unsigned int nMajorCategory, unsigned int nMinorCategory, const char *format, ...);
#else
        static void DebugString(const char *format, ...);
        static void TraceString(const char *format, ...);
#endif


        //Debug StringForUser
        static void DebugStringUserTpServer(const char *format, ...);
        static void DebugStringUserMBusSlave(const char *format, ...);
        static void DebugStringUserIE(const char *format, ...);
        static void DebugStringUser5gTelecom(const char *format, ...);
        static void DebugStringUserUdpServer(const char *format, ...);
        //Debug StringForControl
        static void DebugStringCtrlInterpreter(const char *format, ...);
        static void DebugStringCtrlMotion(const char *format, ...);
        static void DebugStringCtrlProgram(const char *format, ...);
        static void DebugStringCtrlIo(const char *format, ...);
        static void DebugStringCtrlDIo(const char *format, ...);
        static void DebugStringCtrlAIo(const char *format, ...);
        static void DebugStringCtrlConfig(const char *format, ...);
        static void DebugStringCtrlState(const char *format, ...);
        //Debug StringForDevice
        static void DebugStringDeviceMBusMaster(const char *format, ...);
        static void DebugStringDeviceECAT(const char *format, ...);
        static void DebugStringDeviceSerial(const char *format, ...);
        //Debug StringForSystem
        static void DebugStringSysConfig(const char *format, ...);
        static void DebugStringSysFileManager(const char *format, ...);
        //Debug StringForFramework
        static void DebugStrinFrwTask(const char *format, ...);
        static void DebugStrinFrwFile(const char *format, ...);
        static void DebugStrinFrwMBus(const char *format, ...);
        static void DebugStrinFrwSerial(const char *format, ...);
        static void DebugStrinFrwTFTP(const char *format, ...);
        static void DebugStrinFrwSystem(const char *format, ...);
        static void DebugStrinFrwIPC(const char *format, ...);
        static void DebugStrinFrwECAT(const char *format, ...);
        static void DebugStrinFrwParser(const char *format, ...);
        static void DebugStrinFrwSocket(const char *format, ...);


        /*
            windows 와 Linux 의 일반 쓰레드에서 사용하는 일반 Sleep
            Xenomai RT쓰레드가 아닌 일반 쓰레드에서 rt_task_sleep()을 사용하면 동작하지 않아 반드시 본 함수 사용!
        */
        static void SleepX(const int nMilliSec)
        {
#if defined (__XENO__) && defined(DRL_RT_PYTHON)
            rt_task_sleep(nMilliSec * 1e6);
#elif defined (OS_LINUX)
            usleep(nMilliSec * 1000);
#endif
        };

        static void MicroSleepX(const int nMicroSec)
        {
#if defined (__XENO__) && defined(DRL_RT_PYTHON)
            rt_task_sleep(nMicroSec * 1e3);
#elif defined (OS_LINUX)
            usleep(nMicroSec);
#endif
        };

        /*
         * Asrock GPIO Access API
         */
        static int GetAsrockGpio(int nPin);
        static void SetAsrockGpio(int nPin, int nValue);
    };

    class CTaskTimeMeasure
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////
        CTaskTimeMeasure();
        virtual~CTaskTimeMeasure();

        enum {
            TIME_SYNC,
            TIME_ECAT,
            TIME_INPT,
            TIME_CTRL,
            TIME_OUPT,
            TIME_LAST
        };

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////
        static void BeginEtherCATTime();
        static void EndEtherCATTime();

        static void BeginSyncTime();
        static void EndSyncTime();

        static void BeginReadTime();
        static void EndReadTime();

        static void BeginControlTime();
        static void EndCtrlDataime();

        static void BeginWriteTime();
        static void EndWriteTime();

#if defined (__XENO__)
        static SRTIME m_lTickTime[TIME_LAST];
        static SRTIME m_lExecTime[TIME_LAST];
#endif
    };
}
