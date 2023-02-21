#include "TaskInterface.h"
#include "TaskDefines.h"
#include <stdio.h>
using namespace DRAFramework;

CTaskInterface::CTaskInterface(bool bRealTime/* = true*/)
    :m_hCycleSync(bRealTime)
{
    m_bTaskStop         = false;
    m_bIsCyclicTask     = true;
    m_bIsTaskRunning    = false;
    m_bIsTaskStarted    = false;

#ifdef __XENO__
    m_bRealTime         = bRealTime;
#else
    m_bRealTime         = false;
#endif
    m_dCycleInSecond    = 0.;
    m_iTaskPriority     = 99;
    m_iTaskCpuId        = 0;

    m_lpTaskFunc       = NULL;
    m_lpTaskParam      = NULL;
}

CTaskInterface::~CTaskInterface()
{

}

bool CTaskInterface::OpenTask(int iTaskPriority /* = 99 */, int iTaskCpuId /* = 0 */, bool bIsCyclicTask /* = true */, double dCycleInSecond /* = 1e-3 */, char *pName /* = NULL */, int nStackSize /* = TASK_STACK_SIZE */)
{
    bool bResult        = true;

    m_bIsCyclicTask     = bIsCyclicTask;
    m_dCycleInSecond    = dCycleInSecond;

    m_iTaskPriority     = iTaskPriority;
    m_iTaskCpuId        = iTaskCpuId;

    /* Copy task name */
    if (pName != NULL)
        sprintf(m_taskName, "%s", pName);

    if (m_bRealTime) {
#ifdef __XENO__
        void (*OnTaskFunc)(void*);
        OnTaskFunc = m_bIsCyclicTask ? OnCyclicTaskRT : OnNonCyclicTaskRT;

        if (rt_task_spawn (&m_hTaskRT, m_taskName, nStackSize, iTaskPriority, T_CPU(iTaskCpuId)|T_SUSP|T_JOINABLE, OnTaskFunc, this) != 0) {
            bResult = false;
        }
#endif
    }
    else {
        m_hCycleSync.CreateSemapahore(0, 1);
        m_hTask.setName(m_taskName);
        m_hTask.setStackSize(nStackSize);
        m_hTask.setOSPriority(iTaskPriority, SCHED_FIFO);
    }

    if ( !bResult) {
        StopTask();
        bResult = false;
    }

    return bResult;
}


bool CTaskInterface::OpenTask(void (*OnTaskFunc)(void*), void* lpParam, int iTaskPriority /* = 99 */, int iTaskCpuId /* = 0 */, char *pName /* = NULL */, int nStackSize /* = TASK_STACK_SIZE */)
{
    bool bResult        = true;

    m_iTaskPriority     = iTaskPriority;
    m_iTaskCpuId        = iTaskCpuId;

    /* Copy task name */
    if (pName != NULL)
        sprintf(m_taskName, "%s", pName);

    if (m_bRealTime) {
#ifdef __XENO__
        if (rt_task_spawn (&m_hTaskRT, m_taskName, nStackSize, iTaskPriority, T_CPU(iTaskCpuId)|T_SUSP|T_JOINABLE , OnTaskFunc, lpParam) != 0) {
            bResult = false;
        }
#endif
    }
    else {
        m_lpTaskFunc        = OnTaskFunc;
        m_lpTaskParam       = lpParam;

        m_hCycleSync.CreateSemapahore(0, 1);
        m_hTask.setName(m_taskName);
        m_hTask.setStackSize(nStackSize);
        m_hTask.setOSPriority(iTaskPriority, SCHED_FIFO);
    }
    if ( !bResult) {
        StopTask();
        bResult = false;
    }

    return bResult;
}

bool CTaskInterface::StartTask()
{
    bool bResult        = OnInit();

    if (bResult) {

        if (m_bRealTime) {
#ifdef __XENO__
            if (rt_task_resume(&m_hTaskRT) != 0) {
                bResult = false;
            }

            if ( !bResult) {
                StopTask();
            }
#endif
        }
        else {
#ifdef USING_THREAD_ADAPTER
            if (m_lpTaskFunc) {
                m_hTask.start(m_lpTaskFunc, m_lpTaskParam);
            }
            else {
                m_hTask.start(*this);
            }
#else
            m_hTask.start(OnTaskNRT, this);
#endif
        }
    }
    return m_bIsTaskStarted = bResult;
}

void CTaskInterface::StopTask()
{
    if (m_bTaskStop) return;

    m_bTaskStop = true;

    if ( !m_bRealTime) {
        m_hCycleSync.SignalSemaphore();
    }

    OnStop();
}

void CTaskInterface::CloseTask()
{
    if (m_bRealTime) {
#ifdef __XENO__
        if ( !m_bIsTaskStarted) {
            rt_task_delete(&m_hTaskRT);
        }
        else {
            rt_task_join(&m_hTaskRT);
        }
#endif
    }
    else {
        if (m_bIsTaskStarted) {
            m_hTask.join(); //<-- 생성자에서 사용하면 R6025에러 발생
        }
    }
}

void CTaskInterface::SwitchToTask()
{
    if (m_bRealTime) {
#ifdef __XENO__
        rt_task_yield();
#endif
    }
    else {
        m_hTask.yield();
        //YieldProcessor();
    }
}

void CTaskInterface::SleepTask(double dCycleInSecond/* = 1e-3*/)
{
    if (m_bRealTime) {
#ifdef __XENO__
        rt_task_sleep(dCycleInSecond * TIME_NANOSECOND);
#endif
    }
    else {
        m_hTask.sleep(long(dCycleInSecond * TIME_MILISECOND));
    }
}

#ifdef __XENO__
void CTaskInterface::OnCyclicTaskRT(void *pArg)
{
    //try {
    CTaskInterface* pThis = (CTaskInterface*)pArg;
    unsigned long lOverRun = 0;

    pThis->m_bIsTaskRunning  = true;

    rt_task_set_periodic(NULL, TM_NOW, pThis->m_dCycleInSecond * TIME_NANOSECOND);
    while ( !pThis->m_bTaskStop) {
        // synchronization
        if (rt_task_wait_period(&lOverRun) == -ETIMEDOUT) {
            rt_printf("###over-runs(%s, %d)###\n", pThis->m_taskName, lOverRun);
        }
        // main loop
        pThis->OnLoop();
    }
    // clean up
    pThis->OnExit();
    // notify thread state
    pThis->m_bIsTaskRunning = false;

    //} catch (abi::__forced_unwind&) { throw; }
}

void CTaskInterface::OnNonCyclicTaskRT(void *pArg)
{
    //try{
    CTaskInterface* pThis = (CTaskInterface*)pArg;

    pThis->m_bIsTaskRunning  = true;

    while ( !pThis->m_bTaskStop) {
        // main loop
        pThis->OnLoop();
        // scheduled
        pThis->SwitchToTask();
    }
    // clean up
    pThis->OnExit();
    // notify thread state
    pThis->m_bIsTaskRunning = false;

    //} catch (abi::__forced_unwind&) { throw; }
}
#endif

#ifdef USING_THREAD_ADAPTER
void CTaskInterface::run()
{
    // set cpu affinity
    SetCpuAffinity(m_iTaskCpuId);
    // run task
    OnTask();
}
#else
void CTaskInterface::OnTaskNRT(void *pArg)
{
    CTaskInterface* pThis = (CTaskInterface*)pArg;

    // set cpu affinity
    pThis->SetCpuAffinity(pThis->m_iTaskCpuId);

    // run task
    if (pThis->m_lpTaskFunc != NULL)
        pThis->m_lpTaskFunc(pThis->m_lpTaskParam);
    else
        pThis->OnTask();
}
#endif

void CTaskInterface::OnTask()
{
    m_bIsTaskRunning  = true;

    while ( !m_bTaskStop) {

        if (m_bIsCyclicTask) {
            // synchronization
            if (m_hCycleSync.WaitSemaphore(m_dCycleInSecond))   // 종료
                continue;
            else {                             // 주기 수행
                // main loop
                OnLoop();
            }
        }
        else {
            // main loop
            OnLoop();
            // scheduled
            SwitchToTask();
        }
    }
    // clean up
    OnExit();
    // notify thread state
    m_bIsTaskRunning = false;
}

double CTaskInterface::GetCycleInSecond()
{
    return m_dCycleInSecond;
}

bool CTaskInterface::IsTaskRunning()
{
    return m_bIsTaskRunning;
}

bool CTaskInterface::IsTaskStarted()
{
    return m_bIsTaskStarted;
}

void CTaskInterface::SetTaskPriority(int iTaskPriority)
{
    if (m_bRealTime) {
#ifdef __XENO__
        rt_task_set_priority(NULL, iTaskPriority);
#endif
    }
    else {
        m_hTask.setOSPriority(iTaskPriority, SCHED_FIFO);
    }
}

void CTaskInterface::SetCpuAffinity(int iTaskCpuId)
{
    if (m_bRealTime) {

    }
    else {
#if defined (OS_LINUX)
        //sched_setaffinity(
        unsigned long mask = 1 << iTaskCpuId;
        /* bind process to processor 0 */
        pthread_setaffinity_np(pthread_self(), sizeof(mask), (cpu_set_t *)&mask);
#else
        SetThreadAffinityMask(GetCurrentThread(),  1 << iTaskCpuId);
#endif
    }
}
