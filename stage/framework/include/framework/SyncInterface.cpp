#include "SyncInterface.h"
#include "TaskDefines.h"

using namespace DRAFramework;

CSemaphore::CSemaphore(bool bRealTime/* = true*/)
{
#ifdef __XENO__
    m_bRealTime     = bRealTime;
#else
    m_bRealTime     = false;
#endif

    m_pSemaphore = NULL;

    m_lMaximumCount = 0;
    m_lInitialCount = 0;
}

CSemaphore::~CSemaphore()
{
    DeleteSemaphore();
}

bool CSemaphore::CreateSemapahore(int lInitialCount /* = 0 */, int lMaximumCount /* = INT_MAX */)
{
    bool bResult = true;

    m_lMaximumCount = lMaximumCount;
    m_lInitialCount = lInitialCount;
    if (m_bRealTime) {
#ifdef __XENO__
        CHK_XENO_CALL();
        static int nCreateCount = 0;
        sprintf(m_szName, "sync_sem%d", ++nCreateCount);
        bResult = (rt_sem_create(&m_tSempahore, m_szName, lInitialCount, S_FIFO) == 0);
#endif
    }
    else {
        try {
            m_pSemaphore = new Poco::Semaphore(lInitialCount, lMaximumCount);
        } catch (Poco::Exception& e) {
            printf("%s\n", e.displayText().c_str());
            bResult = false;
        }
    }
    return bResult;
}

void CSemaphore::DeleteSemaphore()
{
    if (m_bRealTime) {
#ifdef __XENO__
        CHK_XENO_CALL();
        rt_sem_delete(&m_tSempahore);
#endif
    }
    else {
        if (m_pSemaphore) {
            delete m_pSemaphore;
            m_pSemaphore = NULL;
        }
    }
}

bool CSemaphore::WaitSemaphore(double dTimeoutInSecond)
{
    return P(dTimeoutInSecond);
}

void CSemaphore::SignalSemaphore()
{
    V();
}

bool CSemaphore::P(double dTimeoutInSecond)
{
    bool bResult = true;

    if(m_bRealTime) {
#ifdef __XENO__
        CHK_XENO_CALL();
        bResult = (rt_sem_p(&m_tSempahore, dTimeoutInSecond * TIME_NANOSECOND) == 0);
#endif
    }
    else {
        dTimeoutInSecond *= TIME_MILISECOND;

        try {
            if (dTimeoutInSecond == 0) {
                m_pSemaphore->wait();
            }
            else {
#ifdef _WIN32
				bResult = m_pSemaphore->tryWait(0);
#else
				bResult = m_pSemaphore->tryWait((long)dTimeoutInSecond);
#endif
            }
        }catch(...) {
            bResult = false;
        }
    }

    if (bResult) m_lInitialCount--;

    return bResult;
}

void CSemaphore::V()
{
    if (m_bRealTime) {
#ifdef __XENO__
        CHK_XENO_CALL();
        rt_sem_v(&m_tSempahore);
#endif
    }
    else {
        try {
            if (m_lInitialCount < m_lMaximumCount) {
                m_pSemaphore->set();
                m_lInitialCount++;
            }
        }
        catch (Poco::SystemException& e) {
            printf("%s\n", e.displayText().c_str());
        }
        catch(...) {

        }
    }
}


CMutex::CMutex(bool bRealTime/* = true*/)
{
#ifdef __XENO__
    m_bRealTime     = bRealTime;
#else
    m_bRealTime     = false;
#endif
}

CMutex::~CMutex()
{
    DeleteMutex();
}

bool CMutex::CreateMutex()
{
    bool bResult = true;
    if (m_bRealTime) {
#ifdef __XENO__
        CHK_XENO_CALL();
        static int nCreateCount = 0;
        sprintf(m_szName, "sync_mutex%d", ++nCreateCount);
        bResult = (rt_mutex_create(&m_tMutex, m_szName) == 0);
#endif
    }
    return bResult;
}

void CMutex::DeleteMutex()
{
    if (m_bRealTime) {
#ifdef __XENO__
        //CHK_XENO_CALL();
        rt_mutex_delete (&m_tMutex);
#endif
    }
}

void CMutex::Lock(double dTimeoutInSecond/* = 0*/)
{
    if (m_bRealTime) {
#ifdef __XENO__
        CHK_XENO_CALL();
        if (dTimeoutInSecond == 0)
            rt_mutex_acquire(&m_tMutex, TM_INFINITE);
        else
            rt_mutex_acquire(&m_tMutex, dTimeoutInSecond * TIME_NANOSECOND);
#endif
    }
    else {
        m_Mutex.lock();
    }
}

void CMutex::Unlock()
{
    if (m_bRealTime) {
#ifdef __XENO__
        CHK_XENO_CALL();
        rt_mutex_release(&m_tMutex);
#endif
    }
    else {
        m_Mutex.unlock();
    }
}


CCondition::CCondition(bool bRealTime /* = true */)
{
#ifdef __XENO__
    m_bRealTime     = bRealTime;
#else
    m_bRealTime     = false;
#endif
    if (m_bRealTime) {
        assert(false);
    }

}

CCondition::~CCondition()
{
    DeleteCondition();
}

bool CCondition::CreateCondition()
{
    return true;
}

void CCondition::DeleteCondition()
{
}

void CCondition::Wait(CMutex& mutext, double dTimeoutInSecond)
{
    dTimeoutInSecond *= TIME_MILISECOND;
    try {
        if (dTimeoutInSecond == 0) {
            m_Conditon.wait(mutext.m_Mutex);
        }
        else {
            m_Conditon.tryWait(mutext.m_Mutex, (long)dTimeoutInSecond);
        }
    }catch(...) {

    }
}

void CCondition::Singal()
{
    m_Conditon.signal();
}

CEvent::CEvent(bool bRealTime /* = true */)
{
#ifdef __XENO__
    m_bRealTime     = bRealTime;
#else
    m_bRealTime     = false;
#endif
    m_pEvent = NULL;
}

CEvent::~CEvent()
{
    DeleteEvent();
}

bool CEvent::CreateEvent(char* pszName/* = NULL*/)
{
    bool bResult = true;

    if (m_bRealTime) {
#ifdef __XENO__
        CHK_XENO_CALL();
        bResult = (rt_event_create(&m_tEvnet, pszName, 0, EV_FIFO) == 0);
#endif
    }
    else {
        try {
            m_pEvent = new Poco::Event(false);
        } catch (Poco::Exception& e) {
            printf("%s\n", e.displayText().c_str());
            bResult = false;
        }
    }

    return bResult;
}

void CEvent::DeleteEvent()
{
    if (m_bRealTime) {
#ifdef __XENO__
        CHK_XENO_CALL();
        rt_event_delete(&m_tEvnet);
#endif
    }
    else {
        if (m_pEvent) {
            delete m_pEvent;
            m_pEvent = NULL;
        }
    }
}

bool CEvent::WaitEvent(double dTimeoutInSecond)
{
    bool bResult = true;

    if (m_bRealTime) {
#ifdef __XENO__
        CHK_XENO_CALL();
        unsigned long lFlag;
        bResult = (rt_event_wait(&m_tEvnet, 0x01, &lFlag, EV_ALL,  dTimeoutInSecond * TIME_NANOSECOND) == 0);
#endif
    }
    else {
        dTimeoutInSecond *= TIME_MILISECOND;

        if (dTimeoutInSecond == 0) {
            m_pEvent->wait();
        }
        else {
            bResult = m_pEvent->tryWait((long)dTimeoutInSecond);
        }
    }

    return bResult;
}

void CEvent::SetEvent()
{
    if (m_bRealTime) {
#ifdef __XENO__
        CHK_XENO_CALL();
        rt_event_signal(&m_tEvnet, 0x01);
#endif
    }
    else {
        m_pEvent->set();
    }
}

void CEvent::ResetEvent()
{
    if (m_bRealTime) {
#ifdef __XENO__
        CHK_XENO_CALL();
        unsigned long lFlag;
        rt_event_clear(&m_tEvnet, 0x01, &lFlag);
#endif
    }
    else {
        m_pEvent->reset();
    }
}
