////////////////////////////////////////////////////////////////////////////////
// Includes                                                                   //
////////////////////////////////////////////////////////////////////////////////
#include "NDKUser.h"
#include "NDKSocket.h"

#include <assert.h>

using namespace DRAFramework;
////////////////////////////////////////////////////////////////////////////////
// Constructors / Destructor                                                  //
////////////////////////////////////////////////////////////////////////////////

// Constructor.
CNDKUser::CNDKUser()
{
    m_lId = 0;
    m_lpServerSocket = NULL;
}


// Constructor with initialization.
CNDKUser::CNDKUser(long lId, CNDKServerSocket* pServerSocket)
{
    assert((lId > 0) && (pServerSocket != NULL));

    m_lId = lId;
    m_lpServerSocket = pServerSocket;
}


// Destructor.
CNDKUser::~CNDKUser()
{

}


////////////////////////////////////////////////////////////////////////////////
// Attributes                                                                 //
////////////////////////////////////////////////////////////////////////////////

// Gets the Id.
long CNDKUser::GetId() const
{
    return m_lId;
}


// Returns if the socket is the same as specified.
bool CNDKUser::IsSocketEqual(CNDKServerSocket* pServerSocket) const
{
    return m_lpServerSocket == pServerSocket;
}


// Returns if the buffer of the socket is empty.
bool CNDKUser::IsSocketBufferEmpty() const
{
    assert(m_lpServerSocket != NULL);

    bool bResult = true;

    if (m_lpServerSocket != NULL)
        bResult = m_lpServerSocket->IsBufferEmpty();

    return bResult;
}


////////////////////////////////////////////////////////////////////////////////
// Operations                                                                 //
////////////////////////////////////////////////////////////////////////////////

// Sends a message.
bool CNDKUser::SendMessage(CNDKMessage& message)
{
    assert(m_lpServerSocket != NULL);

    bool bResult = true;

    if (m_lpServerSocket != NULL)
        bResult = m_lpServerSocket->SendMessage(message);

    return bResult;
}



// Reads a message.
bool CNDKUser::ReadMessage(CNDKMessage& message)
{
    assert(m_lpServerSocket != NULL);

    bool bResult = true;

    if (m_lpServerSocket != NULL)
        bResult = m_lpServerSocket->ReceiveMessage(message);

    return bResult;
}



// Closes the socket.
void CNDKUser::CloseSocket()
{
    if (m_lpServerSocket != NULL) {

        m_lpServerSocket->ShutDown();
        m_lpServerSocket->Close();

        // no need to delete, automatically gets deleted magically by TCPServerConnectionFactory
        //delete m_lpServerSocket;
        //m_lpServerSocket = NULL
    }
}


////////////////////////////////////////////////////////////////////////////////
// Operators                                                                  //
////////////////////////////////////////////////////////////////////////////////

// Comparison operator.
bool CNDKUser::operator==(const CNDKUser& user) const
{
    return (m_lId == user.m_lId) && (m_lpServerSocket == user.m_lpServerSocket);
}
