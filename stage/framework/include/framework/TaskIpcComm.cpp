/*
 * IpcInterface.cpp
 *
 * This class is for IPC (Inter Process Communication) via the shared memory method.
 * A class can support some method for shared memory handling such as create, get, set, and delete.
 * Processes can create new shared memory region using CTaskIpcComm class and share this region for communication between each others.
 *
 *  Created on: 2016. 2. 11.
 *      Author: myungjin.kim
 */

#include "TaskIpcComm.h"
#include "TaskDefines.h"
using namespace DRAFramework;

//////////////////////////////////////////////////////////////////////////////////////
////////// CTaskShmComm Class
//////////////////////////////////////////////////////////////////////////////////////

// Constructor
CTaskShmComm::CTaskShmComm()
{
    shm_id = 0;
    isOpen = false;
    pCreatedShm = NULL;
    shm_size = 0;
}


// Destructor
CTaskShmComm::~CTaskShmComm()
{
    if (isOpen == true)
    {
        DeleteShm();
    }
}


// Open shared memory. If shared memory is not exist, then new shared memory is created.
int
CTaskShmComm::OpenShm(key_t key, size_t size, char *name)
{
    // Check state of shared memory
    if (isOpen != false)
    {
        return (-1);
    }

    // Create mutex for mutual exclusion
    shmMutex.CreateMutex();

    // Create event
    shmEvent.CreateEvent(name);

    // Create new shared memory area
    shm_id = shmget(key, size, IPC_CREAT | 0666);
    if (shm_id == (-1))
    {
        isOpen = false;
        return (-1);
    }

    // Attach shared memory area into process memory
    pCreatedShm = (char *)shmat(shm_id, (void *)0, 0);
    if ((int)pCreatedShm == (-1))
    {
        isOpen = false;
        return (-1);
    }

    // Initialize shared memory space
    memset((void *)pCreatedShm, 0, size);

    // Open success and store size information of shared memory region
    isOpen = true;
    shm_size = size;

    CTaskUtil::DebugStrinFrwIPC("Initialize shared memory success\n");

    return (0);
}


// Set specific value to shared memory area
int
CTaskShmComm::SetDataToShm(unsigned int offset, size_t size, char *pData)
{
    // Check state of shared memory
    if (isOpen != true)
    {
        return (-1);
    }

    // Check length of shared memory
    if (shm_size - offset - size < 0)
    {
        return (-1);
    }

    // Set data to shared memory
    shmMutex.Lock();
    memcpy(pCreatedShm + offset, pData, size);
    shmMutex.Unlock();

    // Send event
    shmEvent.SetEvent();

    return (0);
}


// Get data from shared memory
int
CTaskShmComm::GetDataFromShm(unsigned int offset, size_t size, char *pData, double timeout)
{
    bool ret;

    // Check state of shared memory
    if (isOpen != true)
    {
        return (-1);
    }

    // Check length of shared memory
    if (shm_size - offset - size < 0)
    {
        return (-1);
    }

    // Wait event for update shared memory region by other process
    ret = shmEvent.WaitEvent(timeout);
    if (ret == true)
    {
        // Clear event flag
        shmEvent.ResetEvent();

        // Get data from shared memory
        shmMutex.Lock();
        memcpy(pData, pCreatedShm + offset, size);
        shmMutex.Unlock();
    }
    else
    {
        return (-1);
    }

    return (0);
}


// Return start address of shared memory
void *
CTaskShmComm::GetStartAddress(void)
{
    return (pCreatedShm);
}


// Delete shared memory
int
CTaskShmComm::DeleteShm(void)
{
    // Check state of shared memory
    if (isOpen != true)
    {
        return (-1);
    }

    // Delete shared memory region
    shmMutex.Lock();
    struct shmid_ds shmid;
    shmctl(shm_id, IPC_RMID, &shmid);
    shmdt(pCreatedShm);
    shmMutex.Unlock();

    // Delete mutex
    shmMutex.DeleteMutex();

    // Delete event
    shmEvent.DeleteEvent();

    return (0);
}


//////////////////////////////////////////////////////////////////////////////////////
////////// CTaskMsgQComm Class
//////////////////////////////////////////////////////////////////////////////////////

// Constructor
CTaskMsgQComm::CTaskMsgQComm()
{
    isOpen = false;
    msgq_id = 0;
    memset(sndData, 0, MAX_MSGQ_DATA_SIZE);
    memset(rcvData, 0, MAX_MSGQ_DATA_SIZE);
}


// Destructor
CTaskMsgQComm::~CTaskMsgQComm()
{
    if (isOpen == true)
    {
        DeleteMsgQ();
    }
}


// Open a message queue with key value. If a message queue is not exist, then it create new message queue
int
CTaskMsgQComm::OpenMsgQ(key_t key)
{
    // Check state of message queue
    if (isOpen != false)
    {
        return (-1);
    }

    // Create mutex
    msgqMutex.CreateMutex();

    // Open message queue
    msgq_id = msgget(key, IPC_CREAT | 0666);
    if (msgq_id == (-1))
    {
        return (-1);
    }
    else
    {
        isOpen = true;
    }

    return (0);
}


// Send data to message queue
int
CTaskMsgQComm::SendDataToMsgQ(long msgId, void *msg, int msg_size, bool pend)
{
    int ret;
    long *dataType = (long *)&sndData[0];

    // Check state of message queue
    if (isOpen != true)
    {
        return (-1);
    }

    // Check message size
    if ((msg_size > MAX_MSGQ_DATA_SIZE) || (msg_size < 0))
    {
        return (-1);
    }

    // Create data for sending
    msgqMutex.Lock();
    *dataType = msgId;
    memcpy(sndData + sizeof(long), msg, msg_size);
    msgqMutex.Unlock();

    // Send data
    if (pend == false)
    {
        ret = msgsnd(msgq_id, sndData, msg_size, IPC_NOWAIT);
    }
    else
    {
        ret = msgsnd(msgq_id, sndData, msg_size, 0);
    }

    return (ret);
}


// Receive data from message queue
int
CTaskMsgQComm::ReceiveDataFromMsgQ(long msgId, void *msg, int msg_size, bool pend)
{
    int ret;

    // Check state of message queue
    if (isOpen != true)
    {
        return (-1);
    }

    // Check message size
    if ((msg_size > MAX_MSGQ_DATA_SIZE) || (msg_size < 0))
    {
        return (-1);
    }

    // Receive data
    if (pend == false)
    {
        ret = msgrcv(msgq_id, rcvData, msg_size, msgId, IPC_NOWAIT);
    }
    else
    {
        ret = msgrcv(msgq_id, rcvData, msg_size, msgId, 0);
    }

    if (ret != (-1))
    {
        msgqMutex.Lock();
        memcpy(msg, rcvData + sizeof(long), msg_size);
        msgqMutex.Unlock();
    }

    return (ret);
}


int
CTaskMsgQComm::DeleteMsgQ(void)
{
    // Check state of message queue
    if (isOpen != true)
    {
        return (-1);
    }

    // Delete message queue
    msgctl(msgq_id, IPC_RMID, 0);

    // Clear variables
    isOpen = false;
    msgq_id = 0;
    msgqMutex.DeleteMutex();
    memset(sndData, 0, MAX_MSGQ_DATA_SIZE);

    return (0);
}
