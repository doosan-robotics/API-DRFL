#pragma once
#include <stdio.h>
#include <limits.h>

#ifdef __XENO__
#include <native/sem.h>
#include <native/task.h>
#include <native/mutex.h>
#include <native/event.h>
#endif
#include "Poco/Semaphore.h"
#include "Poco/Mutex.h"
#include "Poco/Event.h"
#include "Poco/Exception.h"
#include "Poco/Condition.h"

namespace DRAFramework
{
    class CSemaphore
    {
    public :
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////
        CSemaphore(bool bRealTime = true);
        virtual ~CSemaphore();

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////
        bool CreateSemapahore(int lInitialCount = 0, int lMaximumCount = INT_MAX);
        void DeleteSemaphore();

        bool WaitSemaphore(double dTimeoutInSecond);
        void SignalSemaphore();

        bool P(double dTimeoutInSecond);
        void V();

    private:
        bool                m_bRealTime;
#ifdef __XENO__
        RT_SEM              m_tSempahore;
        char                m_szName[255];
#endif
        Poco::Semaphore*    m_pSemaphore;

        long                m_lInitialCount;
        long                m_lMaximumCount;

    };


    class CMutex
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////
        CMutex(bool bRealTime = true);
        virtual ~CMutex();

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////
        bool CreateMutex();
        void DeleteMutex();

        void Lock(double dTimeoutInSecond = 0);
        void Unlock();

    private:
        friend class CCondition;
        bool                m_bRealTime;
#ifdef __XENO__
        RT_MUTEX            m_tMutex;
        char                m_szName[255];
#endif
        Poco::Mutex         m_Mutex;
    };


    class CCondition
    {
    public :
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////
        CCondition(bool bRealTime = true);
        virtual ~CCondition();

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////
        bool CreateCondition();
        void DeleteCondition();

        void Wait(CMutex& mutext, double dTimeoutInSecond);
        void Singal();

    private:
        bool                m_bRealTime;
        Poco::Condition     m_Conditon;
    };

    class CEvent
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////
        CEvent(bool bRealTime = true);
        virtual ~CEvent();

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////
        bool CreateEvent(char* pszName = NULL);
        void DeleteEvent();

        bool WaitEvent(double dTimeoutInSecond);
        void SetEvent();
        void ResetEvent();

    private:
        bool                m_bRealTime;
#ifdef __XENO__
        RT_EVENT            m_tEvnet;
#endif
        Poco::Event*        m_pEvent;
    };

}
