/*
 * IpcInterface.h
 *
 * This class is for IPC (Inter Process Communication) via the shared memory method.
 * A class can support some methods for shared memory handling such as create, get, set, and delete.
 * Processes can create new shared memory region using CTaskIpcComm class and share this region for communication between each others.
 *
 *  Created on: 2016. 2. 11.
 *      Author: myungjin.kim
 */

#ifndef TASKIPCCOMM_H_
#define TASKIPCCOMM_H_

#pragma once
#include <stdio.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/msg.h>

#include "SyncInterface.h"

#define MAX_MSGQ_DATA_SIZE        (int)1024 - (int)sizeof(long)

namespace DRAFramework
{
    // Class for shared memory
    class CTaskShmComm
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////
        CTaskShmComm();
        ~CTaskShmComm();

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////
        int OpenShm(key_t key, size_t size, char *name);
        int SetDataToShm(unsigned int offset, size_t size, char *pData);
        int GetDataFromShm(unsigned int offset, size_t size, char *pData, double timeout);
        void *GetStartAddress(void);
        int DeleteShm(void);

    private:
        char *pCreatedShm;        // Mapped address in process
        size_t shm_size;        // Size of shared memory region
        int shm_id;                // Key value of shared memory
        bool isOpen;
        CMutex shmMutex;
        CEvent shmEvent;
    };

    // Class for message queue
    class CTaskMsgQComm
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////
        CTaskMsgQComm();
        ~CTaskMsgQComm();

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////
        int OpenMsgQ(key_t key);
        int SendDataToMsgQ(long msgId, void *msg, int msg_size, bool pend = false);
        int ReceiveDataFromMsgQ(long msgId, void *msg, int msg_size, bool pend = false);
        int DeleteMsgQ(void);

    private:
        int msgq_id;            // Message queue id
        bool isOpen;
        CMutex msgqMutex;
        char sndData[MAX_MSGQ_DATA_SIZE];    // Buffer for send data
        char rcvData[MAX_MSGQ_DATA_SIZE];    // Buffer for receive data
    };
}


#endif /* TASKIPCCOMM_H_ */
