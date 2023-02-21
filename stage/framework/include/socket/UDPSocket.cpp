////////////////////////////////////////////////////////////////////////////////
// Includes                                                                   //
////////////////////////////////////////////////////////////////////////////////
#include "UDPSocket.h"

#include <assert.h>
#include <iostream>
using namespace std;
using namespace DRAFramework;

//#define NONBLOCK_SOKCET

#include "../TaskDefines.h"

using namespace DRAFramework;

/////////////////////////////////////////////////////////////////////////////////
// Constructors / Destructor                                                  //
////////////////////////////////////////////////////////////////////////////////

// Constructor
CNDKSocketUDP::CNDKSocketUDP(CNDKServerUDP* lpServer)
{
    m_lpServer          = lpServer;
    m_lpDatagramSocket  = NULL;
    m_bStop             = FALSE;
}

// Destructor.
CNDKSocketUDP::~CNDKSocketUDP()
{
    Close();
}

////////////////////////////////////////////////////////////////////////////////
// Attributes                                                                 //
////////////////////////////////////////////////////////////////////////////////

// Return if the buffer is empty.
bool CNDKSocketUDP::IsBufferEmpty() const
{
    bool bResult = TRUE;

    if (m_lpDatagramSocket)
        bResult = m_lpDatagramSocket->available() > 0 ? FALSE : TRUE;

    return bResult;
}


bool CNDKSocketUDP::Initialize()
{
    if (m_lpDatagramSocket) {
#ifdef NONBLOCK_SOKCET
        // blocking mode 시 라이브러 내에서 send 함수 호출시 간혹 shced_yield 호출함으로
        // Accept 소켓은 모두 비동기 처리하는 것이 맞지 않을까
        m_lpDatagramSocket->setBlocking(false);
#endif
        // 큐가 쌓이면 DRCF 가 "out of memory" 로 죽는걸 예방(타임아웃 적용)
        Poco::Timespan sendTimeout(1, 0); // 1초
        m_lpDatagramSocket->setSendTimeout(sendTimeout);
    }
    return true;
}

void CNDKSocketUDP::Close()
{
    if ( !m_lpDatagramSocket)
        return;

    try {
        m_lpDatagramSocket->close();
    } catch (Poco::Exception &exc) {
        CTaskUtil::DebugStrinFrwSocket("%s\n", exc.displayText().c_str());
    } catch(std::exception& exc) {
        CTaskUtil::DebugStrinFrwSocket("%s\n", exc.what());
    } catch (...) {
        CTaskUtil::DebugStrinFrwSocket("Unknown Exception\n");
    }

    SAFE_DELETE_NEW(m_lpDatagramSocket);
}

void CNDKSocketUDP::ShutDown()
{
    m_bStop = TRUE;
}

// return IP address of remote client
void CNDKSocketUDP::GetPeerAddress(std::string& rIpAddress, unsigned int& rPort)
{
    if (m_lpDatagramSocket != NULL) {

        Poco::Net::SocketAddress sa = m_lpDatagramSocket->peerAddress();

        rIpAddress = sa.host().toString();
        rPort      = sa.port();
    }
}

////////////////////////////////////////////////////////////////////////////////
// Operations                                                                 //
////////////////////////////////////////////////////////////////////////////////

// Sends a message.
bool CNDKSocketUDP::SendMessage(Poco::Net::SocketAddress& rAddress, CNDKMessage& rMessage)
{
    bool bResult =false;

    if ( !m_lpDatagramSocket) {
        return bResult;
    }

    int iTotalByteSend = rMessage.GetLength();
    int iTotalByteSent = 0;
    do
    {
        try {
            int iByteSent = m_lpDatagramSocket->sendTo((rMessage.GetBuffer() + iTotalByteSent), iTotalByteSend - iTotalByteSent, rAddress);
            if (iByteSent > 0) {
                iTotalByteSent += iByteSent;
                bResult = true;
            }
            else{
                bResult = false;
            }

        } catch (Poco::Exception &exc) {
#ifdef NONBLOCK_SOKCET
            CTaskUtil::DebugString("%s\n", exc.displayText().c_str());

            if (exc.code() == POCO_EINTR || exc.code() == POCO_EWOULDBLOCK) {
                bResult = true;
            }
            else {
                bResult = false;
            }
#else
            CTaskUtil::DebugStrinFrwSocket("%s\n", exc.displayText().c_str());
            bResult = false;
#endif
        } catch(std::exception& exc) {
            CTaskUtil::DebugStrinFrwSocket("%s\n", exc.what());
            bResult = false;
        } catch (...) {
            CTaskUtil::DebugStrinFrwSocket("Unknown Exception\n");
            bResult = false;
        }
    } while (bResult && iTotalByteSent != iTotalByteSend);

    return bResult;
}

// Receive a message.
bool CNDKSocketUDP::ReceiveMessage(Poco::Net::SocketAddress& rAddress, CNDKMessage& rMessage)
{
    bool bResult = false;

    if( !m_lpDatagramSocket) {
        return bResult;
    }
    unsigned char szBuffer[READ_BUFFER_SIZE + 1] = {'\0', };

    try {
        int iBytesMessage = m_lpDatagramSocket->receiveFrom(szBuffer, READ_BUFFER_SIZE, rAddress);
        if (iBytesMessage > 0) {
            rMessage.SetBuffer(szBuffer, iBytesMessage);
            bResult = true;
        }
        else {
            CTaskUtil::DebugStrinFrwSocket(" socket closed(receiveFrom length = %d)", iBytesMessage);
            bResult = false;
        }
    } catch (Poco::Exception &exc) {
#ifdef NONBLOCK_SOKCET
        CTaskUtil::DebugString("%s\n", exc.displayText().c_str());

        if (exc.code() == POCO_EINTR || exc.code() == POCO_EWOULDBLOCK) {
            bResult = true;
        }
        else {
            bResult = false;
        }
#else
        CTaskUtil::DebugStrinFrwSocket("%s: %s\n", exc.displayText().c_str(), rAddress.toString().c_str());
        bResult = false;
#endif
    } catch(std::exception& exc) {
        CTaskUtil::DebugStrinFrwSocket("%s\n", exc.what());
        bResult = false;
    } catch (...) {
        CTaskUtil::DebugStrinFrwSocket("Unknown Exception\n");
        bResult = false;
    }

    return bResult;
}

void CNDKSocketUDP::run()
{
    bool bResult = true;
    Poco::Timespan recvTimeout(1, 0); // 1초

    m_evTask.set();

    while ( !m_bStop) {

        try {
            if (m_lpDatagramSocket->poll(recvTimeout, Poco::Net::Socket::SELECT_READ/*|Poco::Net::Socket::SELECT_ERROR*/)) {
                // send 함수 실패시 shutdown 명령으로 소켓 버퍼가 리셋 됨으로
                // recevie 함수에서 버퍼에서 읽을 내용이 없어 LOCK 걸리는 것을 방지함.
                if (m_bStop) break;

                //CTaskUtil::DebugStrinFrwSocket("m_lpDatagramSocket->poll() return");
                OnReceive(0);
            }
        } catch (Poco::Exception &exc) {
            CTaskUtil::DebugStrinFrwSocket("%s\n", exc.displayText().c_str());
            bResult = false;
        } catch(std::exception& exc) {
            CTaskUtil::DebugStrinFrwSocket("%s\n", exc.what());
            bResult = false;
        } catch (...) {
            CTaskUtil::DebugStrinFrwSocket("Unknown Exception\n");
            bResult = false;
        }

        if ( !bResult) break;
    }
    CTaskUtil::DebugStrinFrwSocket("The thread of RTServer is Closed\n");
}

#ifdef __XENO__
int  CNDKServerSocketUDP::m_iRTTask = 0;
#endif
////////////////////////////////////////////////////////////////////////////////
// Constructors / Destructor                                                  //
////////////////////////////////////////////////////////////////////////////////

// Constructor.
CNDKServerSocketUDP::CNDKServerSocketUDP(CNDKServerUDP* lpServer)
    : CNDKSocketUDP(lpServer)
{

}

// Destructor.
CNDKServerSocketUDP::~CNDKServerSocketUDP()
{

}

bool CNDKServerSocketUDP::Start(int iPort)
{
    bool bResult = TRUE;

    if (m_lpDatagramSocket !=NULL) return bResult;

    try {
        // create socket
        Poco::Net::SocketAddress sa(Poco::Net::IPAddress(), (unsigned short)iPort);
        m_lpDatagramSocket = new Poco::Net::DatagramSocket(sa, TRUE);
        // crate thread to read socket asynchronously
        m_hTask.start(*this);
        m_evTask.wait();

        CTaskUtil::DebugStrinFrwSocket("RTServer is started\n");

    } catch (Poco::Exception &exc) {
        CTaskUtil::DebugStrinFrwSocket("%s\n", exc.displayText().c_str());
        bResult = FALSE;
    } catch(std::exception& exc) {
        CTaskUtil::DebugStrinFrwSocket("%s\n", exc.what());
        bResult = FALSE;
    } catch (...) {
        CTaskUtil::DebugStrinFrwSocket("Unknown Exception\n");
        bResult = FALSE;
    }

    if ( !bResult) { Stop(); }

    return bResult;
}

void CNDKServerSocketUDP::run()
{
#ifdef __XENO__
    char szTaskName[256] = {'\0', };
    sprintf(szTaskName, "siRTServer#%d", m_iRTTask++);

    if (rt_task_self() == 0) {
        rt_task_shadow(&m_hRTTask, szTaskName, PRIORITY_HIGH, T_CPU(USP_CPU));
    }
#endif
    CNDKSocketUDP::run();
}

// Stops the server.
void CNDKServerSocketUDP::Stop()
{
    ShutDown();

    m_hTask.join();
    // should close socket after thread is closed
    Close();
}


// Called when data is received.
void CNDKServerSocketUDP::OnReceive(int nErrorCode)
{
    CNDKSocketUDP::OnReceive(nErrorCode);

    if (m_lpServer != NULL) {
        m_lpServer->ProcessPendingRead(this, nErrorCode);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Constructors / Destructor                                                  //
////////////////////////////////////////////////////////////////////////////////

// Constructor.
CNDKServerUDP::CNDKServerUDP(void)
{
    m_lpServer          = NULL;
    m_lNextId           = 0;
    m_iPort             = -1;
}

// Destructor.
CNDKServerUDP::~CNDKServerUDP(void)
{
    SAFE_DELETE_NEW(m_lpServer);
}


////////////////////////////////////////////////////////////////////////////////
// Attributes                                                                 //
////////////////////////////////////////////////////////////////////////////////

// Returns if the server is started.
bool CNDKServerUDP::IsStarted() const
{
    return m_lpServer != NULL;
}

// Returns the port.
int CNDKServerUDP::GetPort() const
{
    return m_iPort;
}

// Returns the number of connected users.
long CNDKServerUDP::GetNbUsers() const
{
    return (long)m_mgrCtrlIds.size();
}

// Gets a socket from an Id.
void CNDKServerUDP::GetPeerAddress(long lUserId, std::string& rIpAddress, unsigned int& rPort)
{
    rIpAddress = m_mgrCtrlIds[lUserId].host().toString();
    rPort      = (unsigned int)m_mgrCtrlIds[lUserId].port();
}

////////////////////////////////////////////////////////////////////////////////
// Operations                                                                 //
////////////////////////////////////////////////////////////////////////////////

// Starts listening on the specified port.
bool CNDKServerUDP::StartListening(int iPort)
{
    bool bResult = false;

    m_iPort = iPort;
    m_lpServer  = new CNDKServerSocketUDP(this);

    bResult = m_lpServer->Start(iPort);
    if ( !bResult) { Stop(); }

    return bResult;
}

// Stops the server.
void CNDKServerUDP::Stop()
{
    if (m_lpServer != NULL) {
        m_lpServer->Stop();
    }
    m_iPort = -1;
}


// Sends a message to a specified user. If a problem occurs, OnDisconnect
// callback will be called.
bool CNDKServerUDP::SendMessageToUser(long lUserId, CNDKMessage& rMessage)
{
    bool bResult = TRUE;

    MAPSOCKMANAGER::iterator it = m_mgrCtrlIds.find(lUserId);
    if ( it != m_mgrCtrlIds.end()) {
        Poco::Net::SocketAddress saClient = it->second;
        bResult = m_lpServer->SendMessage(saClient, rMessage);

        if ( !bResult) { 
            m_mgrCtrlIds.erase(it); 
            OnDisconnect(lUserId);
        }
    }

    return bResult;
}


// Sends a message to all users. OnDisconnect callback will be called for
// each user that the message cannot be sent.
bool CNDKServerUDP::SendMessageToAllUsers(CNDKMessage& rMessage)
{
    LISTSOCKIDS listIds;

    MAPSOCKMANAGER::iterator it = m_mgrCtrlIds.begin();
    for ( ; it != m_mgrCtrlIds.end(); it++) {
        Poco::Net::SocketAddress saClient = it->second;
        if ( !m_lpServer->SendMessage(saClient, rMessage)) {
            listIds.push_back(it->first);
        }
    }

    bool bResult = listIds.empty();

    while (!listIds.empty()) {

        LISTSOCKIDS::iterator itId = listIds.begin();

        MAPSOCKMANAGER::iterator it = m_mgrCtrlIds.find(*itId);
        if ( it != m_mgrCtrlIds.end()) {
            m_mgrCtrlIds.erase(it);
            OnDisconnect(*itId);
        }
        listIds.erase(itId);
    }

    return bResult;
}

// Processes pending read.
bool CNDKServerUDP::ProcessPendingRead(CNDKServerSocketUDP* pServerSocket, long lErrorCode)
{
    bool bResult = FALSE;
    long lSockId = -1;

    MAPSOCKMESSAGE mapIdMessages;
    do {
        CNDKMessage message;
        Poco::Net::SocketAddress saClient;
        bResult = m_lpServer->ReceiveMessage(saClient, message);
        if ( !bResult) break;
        else {
            // Find SocketId
            MAPSOCKMANAGER::iterator it = m_mgrCtrlIds.begin();
            for ( ; it != m_mgrCtrlIds.end(); it++) {
                if (saClient == it->second) {
                    lSockId = it->first;
                    mapIdMessages[lSockId].push_back(message);
                }
            }

            // Add New SockId
            if (lSockId == -1) {
                m_lNextId++;
                m_mgrCtrlIds[m_lNextId] = saClient;
                OnConnect(m_lNextId);

                mapIdMessages[m_lNextId].push_back(message);
            }
        }
    } while ( !m_lpServer->IsBufferEmpty());

    MAPSOCKMESSAGE::iterator itIdMessages= mapIdMessages.begin();
    for ( ; itIdMessages != mapIdMessages.end(); itIdMessages++) {

        LSTMESSAGES lstMessages = itIdMessages->second;

        LSTMESSAGES::iterator itMessages = lstMessages.begin();
        for ( ; itMessages != lstMessages.end(); itMessages++) {
            TranslateMessage(itIdMessages->first, *itMessages);
        }
    }
    mapIdMessages.clear();

    return TRUE;
}

// Translates message and does the appropriate task for message handled by
// the NDK.
void CNDKServerUDP::TranslateMessage(long lSockId, CNDKMessage& message)
{
    OnMessage(lSockId, message);
}
