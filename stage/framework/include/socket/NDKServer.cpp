////////////////////////////////////////////////////////////////////////////////
// Includes                                                                   //
////////////////////////////////////////////////////////////////////////////////
#include "NDKServer.h"
#include "NDKMessage.h"
#include "NDKSocket.h"

#include <assert.h>
#include "../TaskDefines.h"

using namespace DRAFramework;

#ifdef __XENO__
int TCPServerEx::m_iRTTask = 0;
#endif
TCPServerEx::TCPServerEx(Poco::Net::TCPServerConnectionFactory::Ptr pFactory, const Poco::Net::ServerSocket& socket)
    : Poco::Net::TCPServer(pFactory, socket)
{

}

TCPServerEx::~TCPServerEx(void)
{

}

void TCPServerEx::run()
{
#ifdef __XENO__
    char szTaskName[256] = {'\0', };
    sprintf(szTaskName, "siTcpServer#%d", m_iRTTask++);

    if (rt_task_self() == 0) {
        rt_task_shadow(&m_hRTTask, szTaskName, PRIORITY_SOCKET, T_CPU(USR_CPU));
    }
#endif

    TCPServer::run();
}

CNDKServer::CNDKServer(void)
{
    m_iPort             = -1;
    m_lpServer          = NULL;
    m_lpSeverSocket     = NULL;
}


CNDKServer::~CNDKServer(void)
{
   Stop();
}

////////////////////////////////////////////////////////////////////////////////
// Attributes                                                                 //
////////////////////////////////////////////////////////////////////////////////

// Returns if the server is started.
bool CNDKServer::IsStarted() const
{
    return m_lpSeverSocket != NULL;
}

// Returns the port.
int CNDKServer::GetPort() const
{
    return m_iPort;
}

// Returns the number of connected users.
long CNDKServer::GetNbUsers() const
{
    return m_mgrCtrlIds.GetNbUsers();
}

// Returns the Ids of all connected users.
void CNDKServer::GetUserIds(CLongArray& alIds)
{
    m_mgrCtrlIds.GetUserIds(alIds);
}

CNDKServerSocket* CNDKServer::GetUserSocket(long lUserId)
{
    return m_mgrCtrlIds.GetSocketFromId(lUserId);
}

////////////////////////////////////////////////////////////////////////////////
// Operations                                                                 //
////////////////////////////////////////////////////////////////////////////////

// Starts listening on the specified port.
bool CNDKServer::StartListening(int iPort)
{
    // Don't call this method two times, call Stop before another
    // call to StartListening
    assert(m_lpServer == NULL);

    bool bResult = false;

    m_iPort = iPort;

    try
    {
        m_lpSeverSocket     = new Poco::Net::ServerSocket(m_iPort);
        m_lpServer          = new TCPServerEx(new CNDKServerConnectionFactory(this), *m_lpSeverSocket);

        m_lpServer->start();
        bResult = true;

    } catch (Poco::Exception &exc) {
        exc.displayText();
        bResult = false;
    } catch(std::exception& exc) {
        exc.what();
        bResult = false;
    } catch (...) {
        bResult = false;
    }

    if ( !bResult) {
        Stop();
    }

    return bResult;
}

// Stops the server.
void CNDKServer::Stop()
{
    // 하단의 이동: 서버 종료 전에 접속을 끊으면, 클라이언트 재접속 로직에 의해서 다시 접속이 허용되는 오류 수정
    //DisconnectAllUsers();

    if (m_lpServer != NULL) {
        m_lpServer->stop();

        delete m_lpServer;
        m_lpServer = NULL;
    }

    if (m_lpSeverSocket != NULL) {

        delete m_lpSeverSocket;
        m_lpSeverSocket = NULL;
    }
    
#if 0 //no need to delete, automatically gets deleted magically by TCPServerConnectionFactory
    if (GetNbUsers()) {
        CTaskUtil::DebugStrinFrwSocket("Disconnect all clients\n");
        DisconnectAllUsers();
    }
#endif
    m_iPort = -1;
}


// Sends a message to a specified user. If a problem occurs, OnDisconnect
// callback will be called.
bool CNDKServer::SendMessageToUser(long lUserId, CNDKMessage &msg)
{
    bool bResult = m_mgrCtrlIds.SendMessage(lUserId, msg);

    if (!bResult)
        OnDisconnect(lUserId, NDKServer_ErrorSendingMessage);

    return bResult;
}


// Sends a message to all users. OnDisconnect callback will be called for
// each user that the message cannot be sent.
bool CNDKServer::SendMessageToAllUsers(CNDKMessage& message)
{
    CLongList listIds;

    bool bResult = m_mgrCtrlIds.SendMessageToAllUsers(message, listIds);

    while (!listIds.empty()) {
        CLongList::iterator itId = listIds.begin();
        OnDisconnect(*itId, NDKServer_ErrorSendingMessage);
        listIds.erase(itId);
    }
    return bResult;
}


// Sends a message to some user specified by the array of user Id.
// OnDisconnect callback will be called for each user that the message
// cannot be sent.
bool CNDKServer::SendMessageToSomeUsers(const CLongArray& alUserIds, CNDKMessage& message)
{
    bool bResult = true;

    for (unsigned int iUserIndex = 0; iUserIndex < alUserIds.size(); iUserIndex++)
    {
        if (!SendMessageToUser(alUserIds[iUserIndex], message))
        {
            OnDisconnect(alUserIds[iUserIndex], NDKServer_ErrorSendingMessage);
            //bResult = false;
        }
    }

    return bResult;
}


// Sends a message to all users except for the specified user Id.
// OnDisconnect callback will be called for each user that the message
// cannot be sent.
bool CNDKServer::SendMessageToAllUsersExceptFor(long lUserId, CNDKMessage& message)
{
    CLongArray alUserIds;

    alUserIds.push_back(lUserId);

    return SendMessageToAllUsersExceptFor(alUserIds, message);
}


// Sends a message to all users that aren't in the specified array of user
// Id. OnDisconnect callback will be called for each user that the message
// cannot be sent.
bool CNDKServer::SendMessageToAllUsersExceptFor(const CLongArray& alUserIds,
    CNDKMessage& message)
{
    CLongList listIds;

    bool bResult = m_mgrCtrlIds.SendMessageToAllUsersExceptFor(alUserIds, message, listIds);

    while (!listIds.empty()) {
        CLongList::iterator itId = listIds.begin();
        OnDisconnect(*itId, NDKServer_ErrorSendingMessage);
        listIds.erase(itId);
    }

    return bResult;

}


// Disconnects a specified user. OnDisconnection callback will be call with the value
// NDKNormalDisconnection.
bool CNDKServer::DisconnectUser(long lUserId)
{
    return DisconnectUser(lUserId, NDKServer_NormalDisconnection);
}


// Disconnects all users. OnDisconnect callback will not be called for
// users disconnected that way.
void CNDKServer::DisconnectAllUsers()
{
    m_mgrCtrlIds.RemoveAllUsers();
}

////////////////////////////////////////////////////////////////////////////////
// Protected Operations                                                       //
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Private Operations                                                         //
////////////////////////////////////////////////////////////////////////////////

// Processes pending accept.
void CNDKServer::ProcessPendingAccept(CNDKServerSocket* pServerSocket, long lErrorCode)
{
    if (lErrorCode == 0) {

        bool bResult = false;

        // Ask to accept the new connection
        if (pServerSocket->Initialize() && OnIsConnectionAccepted()) {

            OnConnect(m_mgrCtrlIds.AddUser(pServerSocket));

            bResult = true;
        }

        if ( !bResult) {
            pServerSocket->Close();
        }
    }
}

// Processes pending read.
bool CNDKServer::ProcessPendingRead(CNDKServerSocket* pServerSocket, long lErrorCode)
{
    bool            bIsConnectionOpened = true;
    long            lSockId = 0;
    CNDKMessageList messages;

    bool bResult = m_mgrCtrlIds.ProcessPendingRead(pServerSocket, lErrorCode, lSockId, messages);

    if (bResult) {

        CNDKMessageList::iterator it = messages.begin();

        while (it != messages.end())
            TranslateMessage(lSockId, *it++);

        messages.clear();
    }
    else {

        DisconnectUser(lSockId, NDKServer_ErrorReceivingMessage);
        bIsConnectionOpened = false;
    }

    return bIsConnectionOpened;
}

// Translates message and does the appropriate task for message handled by
// the NDK.
void CNDKServer::TranslateMessage(long lSockId, CNDKMessage& message)
{
    OnMessage(lSockId, message);
}

// Disconnects a user.
bool CNDKServer::DisconnectUser(long lSockId, NDKServerDisconnection disconnectionType)
{
    bool bResult = m_mgrCtrlIds.RemoveUser(lSockId);

    OnDisconnect(lSockId, disconnectionType);

    return bResult;
}
