////////////////////////////////////////////////////////////////////////////////
// Includes                                                                   //
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include <assert.h>
#include <string.h>
#ifdef __XENO__
#include <native/queue.h>
#endif
#include "SyncInterface.h"

#ifndef SAFE_DELETE_NEW
#define SAFE_DELETE_NEW(p)  {   if (p) {    delete p;   p = NULL;   }   }
#endif

#if defined (__XENO__)
// memory consumption(rt_queue_wrte) and unproven performance issue
//#define USE_XENO_QUEUE
#endif

namespace DRAFramework {

    ////////////////////////////////////////////////////////////////////////////////
    // Template Class                                                             //
    ////////////////////////////////////////////////////////////////////////////////

    template<typename QUEUE_DATA, int QUEUE_SIZE>
    class CTaskQueue {

    public:

        enum {
            eQUEUE_SIZE = QUEUE_SIZE,
        };

        enum {
            MUTEX_ENQUEUE,
            MUTEX_DEQUEUE,
            MUTEX_SZQUEUE,
            MUTEX_LAST,
        };

        CTaskQueue(bool bRealTime = true) : m_smConsumer(bRealTime) , m_smProducer(bRealTime)
        {
            m_iHead             = 0;
            m_iTail             = 0;
            m_iSize             = 0;

            m_dSyncTimeOut      = 0;
            m_bSyncQueue        = false;

            m_bStop             = false;
#ifdef __XENO__
            m_bRealTime         = bRealTime;
#else
            m_bRealTime         = false;
#endif

            m_nDataSize         = sizeof(QUEUE_DATA);

            for (int i = 0; i < eQUEUE_SIZE; i++)
                m_pQueue[i] = NULL;

            for (int i = 0; i < MUTEX_LAST; i++)
                m_pQueueLock[i] = NULL;
        };

        virtual~ CTaskQueue()
        {
#if defined (USE_XENO_QUEUE)
            rt_queue_flush(&m_tXeQueue);
            rt_queue_delete(&m_tXeQueue);
#else
            for (int i = 0; i < MUTEX_LAST; i++)
                SAFE_DELETE_NEW(m_pQueueLock[i]);

            for (int i = 0; i < eQUEUE_SIZE; i++)
                SAFE_DELETE_NEW(m_pQueue[i]);
#endif
        };

        //
        bool OpenQueue(bool bSyncQueue = false, double dSyncTimeOut = 0.) {

            bool bResult = false;

            m_bSyncQueue     = bSyncQueue;
            m_dSyncTimeOut   = dSyncTimeOut;

            if(m_bSyncQueue) {
                m_smConsumer.CreateSemapahore(0,           eQUEUE_SIZE);
                m_smProducer.CreateSemapahore(eQUEUE_SIZE, eQUEUE_SIZE);
            }

#if defined (USE_XENO_QUEUE)
            static int nCreateCount = 0;
            sprintf(m_szName, "sync_queue%d", ++nCreateCount);
            // the function of rt_queue_write use memory in multple of 2
            // if 4116 size memory should be used, the function of rt_queue_write uses 8192 memory
            bResult = (rt_queue_create(&m_tXeQueue, m_szName, eQUEUE_SIZE * m_nDataSize * 2, eQUEUE_SIZE, Q_FIFO) == 0);
#else
            for (int i = 0; i < eQUEUE_SIZE; i++)
                m_pQueue[i] = new QUEUE_DATA;

            for (int i = 0; i < MUTEX_LAST; i++) {
                m_pQueueLock[i] = new CMutex(m_bRealTime);
                m_pQueueLock[i]->CreateMutex();
            }
            bResult = true;
#endif
            return bResult;
        };

        // Inserts an element in queue at rear end
        bool EnQueue(QUEUE_DATA* pQueueData) {

            bool bResult = false;

            if (m_bSyncQueue) {

                if (m_smProducer.WaitSemaphore(m_dSyncTimeOut)) {
                    if ( !m_bStop) {
                        bResult = WriteQueue(pQueueData);
                        m_smConsumer.SignalSemaphore();
                    }
                }
            }
            else {
                if (GetQueueCount() < eQUEUE_SIZE) {
                    if ( !m_bStop) {
                        bResult = WriteQueue(pQueueData);
                    }
                }
            }
            return bResult;
        };

        // Removes an element in Queue from front end.
        bool DeQueue(QUEUE_DATA* pQueueData) {

            bool bResult = false;

            if (m_bSyncQueue) {

                if (m_smConsumer.WaitSemaphore(m_dSyncTimeOut)) {
                    if ( !m_bStop) {
                        bResult = ReadQueue(pQueueData);
                        m_smProducer.SignalSemaphore();
                    }
                }
            }
            else {
                if (GetQueueCount() > 0) {
                    if ( !m_bStop) {
                        bResult = ReadQueue(pQueueData);
                    }
                }
            }
            return bResult;
        };

        QUEUE_DATA* DeQueue() {

            QUEUE_DATA* pQueueData = NULL;

            if (m_bSyncQueue) {

                if (m_smConsumer.WaitSemaphore(m_dSyncTimeOut)) {
                    if ( !m_bStop) {
                        pQueueData = ReadQueue();
                        m_smProducer.SignalSemaphore();
                    }
                }
            }
            else {
                if (GetQueueCount() > 0) {
                    if ( !m_bStop) {
                        pQueueData = ReadQueue();
                    }
                }
            }
            return pQueueData;
        };

        //
        void CloseQueue() {

            m_bStop = true;

            if (m_bSyncQueue) {
                m_smConsumer.SignalSemaphore();
                m_smProducer.SignalSemaphore();
            }
        };

        unsigned int GetQueueCount() {
            // 이정우 수정 2017.5.13 수원
            unsigned int nCount = 0;

#if defined (USE_XENO_QUEUE)
            RT_QUEUE_INFO tInfo = {'\0', };
            rt_queue_inquire(&m_tXeQueue, &tInfo);
            nCount = tInfo.nmessages;
#else
            m_pQueueLock[MUTEX_SZQUEUE]->Lock();
            nCount = m_iSize;
            m_pQueueLock[MUTEX_SZQUEUE]->Unlock();
#endif
            return nCount;
        };

    private:
        inline bool WriteQueue(QUEUE_DATA* pQueueData) {

            bool bResult = true;

#if defined (USE_XENO_QUEUE)
            bResult = (rt_queue_write(&m_tXeQueue, pQueueData, m_nDataSize, Q_NORMAL) >= 0);
#else
            m_pQueueLock[MUTEX_ENQUEUE]->Lock();
            memcpy(m_pQueue[m_iTail], pQueueData, m_nDataSize);
            m_iTail = (m_iTail + 1) % eQUEUE_SIZE;
            m_pQueueLock[MUTEX_ENQUEUE]->Unlock();
            m_pQueueLock[MUTEX_SZQUEUE]->Lock();
            m_iSize = (m_iSize + 1);
            m_pQueueLock[MUTEX_SZQUEUE]->Unlock();
#endif
            return bResult;
        };

        inline bool ReadQueue(QUEUE_DATA* pQueueData) {

            bool bResult = true;

#if defined (USE_XENO_QUEUE)
            bResult = (rt_queue_read(&m_tXeQueue, pQueueData, m_nDataSize, TM_NONBLOCK) >= 0);
#else
            m_pQueueLock[MUTEX_DEQUEUE]->Lock();
            memcpy(pQueueData, m_pQueue[m_iHead], m_nDataSize);
            m_iHead = (m_iHead + 1) % eQUEUE_SIZE;
            m_pQueueLock[MUTEX_DEQUEUE]->Unlock();
            m_pQueueLock[MUTEX_SZQUEUE]->Lock();
            m_iSize = (m_iSize - 1);
            m_pQueueLock[MUTEX_SZQUEUE]->Unlock();
#endif
            return bResult;
        };

        inline QUEUE_DATA* ReadQueue() {

            bool bResult = true;

#if defined (USE_XENO_QUEUE)
            bResult = (rt_queue_read(&m_tXeQueue, &m_QueueData, m_nDataSize, TM_NONBLOCK) > 0);
#else
            m_pQueueLock[MUTEX_DEQUEUE]->Lock();
            memcpy(&m_QueueData, m_pQueue[m_iHead], m_nDataSize);
            m_iHead = (m_iHead + 1) % eQUEUE_SIZE;
            m_pQueueLock[MUTEX_DEQUEUE]->Unlock();
            m_pQueueLock[MUTEX_SZQUEUE]->Lock();
            m_iSize = (m_iSize - 1);
            m_pQueueLock[MUTEX_SZQUEUE]->Unlock();
#endif
            return bResult ? &m_QueueData : NULL;
        };

#if defined (USE_XENO_QUEUE)
        RT_QUEUE              m_tXeQueue;
        char                  m_szName[255];
#endif
        QUEUE_DATA            m_QueueData;
        QUEUE_DATA            *m_pQueue[eQUEUE_SIZE];
        int                   m_iTail;
        int                   m_iHead;
        int                   m_iSize;
        int                   m_nDataSize;

        bool                  m_bRealTime;
        bool                  m_bStop;
        bool                  m_bSyncQueue;
        double                m_dSyncTimeOut;
        CSemaphore            m_smConsumer;
        CSemaphore            m_smProducer;

        CMutex*               m_pQueueLock[MUTEX_LAST];
    };
}
