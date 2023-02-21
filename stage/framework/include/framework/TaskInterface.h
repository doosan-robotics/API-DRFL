#pragma once

#if defined (OS_LINUX)
#include <unistd.h>
#if defined (__XENO__)
#include <rtdk.h>
#include <native/task.h>
#include <native/timer.h>
#endif
#endif

#include <Poco/Thread.h>
#include <Poco/Runnable.h>

#include "TaskDefines.h"
#include "SyncInterface.h"

//#define USING_THREAD_ADAPTER /* 사용시 우선순위 설정 않됨*/

namespace DRAFramework {

    class CTaskInterface
#ifdef USING_THREAD_ADAPTER
        : public Poco::Runnable
#endif
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////
        CTaskInterface(bool bRealTime = true);
        virtual~CTaskInterface();

        typedef void (*OnTaskFunc)(void*) ;
        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////
        // create task)
        bool OpenTask(int iTaskPriority = 99, int iTaskCpuId = 0, bool bIsCyclicTask = true, double dCycleInSecond = 1e-3, char *pName = NULL, int nStackSize = TASK_STACK_SIZE);
        // create task
        bool OpenTask(void (*OnTaskFunc)(void*), void*  lpParam, int iTaskPriority = 99, int iTaskCpuId = 0, char *pName = NULL, int nStackSize = TASK_STACK_SIZE);
        // start task
        bool StartTask();
        // stop task
        void StopTask();
        // destroy task
        void CloseTask();
        // switch task
        void SwitchToTask();
        // sleep task
        void SleepTask(double dCycleInSecond = 1e-3);
        // user-defined task

        ////////////////////////////////////////////////////////////////////////////
        // Attributes                                                             //
        ////////////////////////////////////////////////////////////////////////////
        // get flag about thread state
        bool IsTaskRunning();
        bool IsTaskStarted();

        // get cycle period
        double GetCycleInSecond();
        // set priority
        void SetTaskPriority(int iTaskPriority);

        void SetCpuAffinity(int iTaskCpuId);

    protected:
        ////////////////////////////////////////////////////////////////////////////
        // Callback                                                               //
        ////////////////////////////////////////////////////////////////////////////
        // when StartTask function in called
        virtual bool OnInit() { return true; };
        // when the thread is running
        virtual void OnLoop() {};
        //when StopTask function in called
        virtual void OnStop() {};
        // when thread is closed
        virtual void OnExit() {};

    private:
        ////////////////////////////////////////////////////////////////////////////
        // Private Operations                                                     //
        ////////////////////////////////////////////////////////////////////////////

#ifdef __XENO__
        /* cyclic thread function */
        static void OnCyclicTaskRT(void *pArg);
        /* non-cyclic thread function */
        static void OnNonCyclicTaskRT(void *pArg);
#endif
        /* general thread function */
#ifdef USING_THREAD_ADAPTER
        virtual void run();
#else
        static void OnTaskNRT(void *pArg);
#endif
        void OnTask();

    protected:

        bool                m_bTaskStop;
        bool                m_bIsTaskRunning;
        bool                m_bIsTaskStarted;

        bool                m_bIsCyclicTask;
        int                 m_iTaskCpuId;
        int                 m_iTaskPriority;
        double              m_dCycleInSecond;
#ifdef __XENO__
        RT_TASK             m_hTaskRT;
#endif
        CSemaphore          m_hCycleSync;
        Poco::Thread        m_hTask;

        OnTaskFunc          m_lpTaskFunc;
        void*               m_lpTaskParam;
        bool                m_bRealTime;
        char                m_taskName[255];
    };
}
