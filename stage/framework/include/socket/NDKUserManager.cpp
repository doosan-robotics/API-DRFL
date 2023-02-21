////////////////////////////////////////////////////////////////////////////////
// Includes                                                                   //
////////////////////////////////////////////////////////////////////////////////
#include "NDKUserManager.h"
#include "NDKSocket.h"

#include <algorithm>

using namespace DRAFramework;
////////////////////////////////////////////////////////////////////////////////
// Constructors / Destructor                                                  //
////////////////////////////////////////////////////////////////////////////////

// Constructor.
CNDKUserManager::CNDKUserManager()
{
    m_lNextId = 0;
    m_mxUsers.CreateMutex();
}


// Destructor.
CNDKUserManager::~CNDKUserManager()
{
#if 0 //no need to delete, automatically gets deleted magically by TCPServerConnectionFactory
    RemoveAllUsers();
#endif
    m_lNextId = 0;
}


////////////////////////////////////////////////////////////////////////////////
// Attributes                                                                 //
////////////////////////////////////////////////////////////////////////////////

// Gets the number of users.
long CNDKUserManager::GetNbUsers() const
{
    long lUesrs = 0;

    m_mxUsers.Lock();

    lUesrs = (long)m_users.size();

    m_mxUsers.Unlock();

    return lUesrs;
}


// Returns the Ids of all users.
void CNDKUserManager::GetUserIds(CLongArray& alIds) const
{
    alIds.clear();

    m_mxUsers.Lock();

    CUserList::const_iterator itUser = m_users.begin();

    while (itUser != m_users.end()) {

        const CNDKUser& user = *itUser++;
        alIds.push_back(user.GetId());
    }

    m_mxUsers.Unlock();
}


////////////////////////////////////////////////////////////////////////////////
// Operations                                                                 //
////////////////////////////////////////////////////////////////////////////////

// Adds a user then returns its new Id.
long CNDKUserManager::AddUser(CNDKServerSocket* pServerSocket)
{
    m_lNextId++;

    m_mxUsers.Lock();

    m_users.push_back(CNDKUser(m_lNextId, pServerSocket));

    m_mxUsers.Unlock();

    return m_lNextId;
}


// Removes a user.
bool CNDKUserManager::RemoveUser(long lSockId)
{
    bool bResult = false;

    m_mxUsers.Lock();

    CNDKUser user = GetUserFromId(lSockId);

    if (user.GetId() > 0) {

        CUserList::iterator itUser = std::find(m_users.begin(), m_users.end(), user);
        if (itUser != m_users.end()) {

            user.CloseSocket();
            m_users.erase(itUser);
            bResult = true;
        }
    }
    m_mxUsers.Unlock();

    return bResult;
}


// Removes all users.
void CNDKUserManager::RemoveAllUsers()
{
    m_mxUsers.Lock();

    CUserList::iterator itUser = m_users.begin();

    while (itUser != m_users.end()) {

        CNDKUser& user = *itUser;
        user.CloseSocket();
        m_users.erase(itUser++);
    }

    m_users.clear();

    m_mxUsers.Unlock();
}


// Sends a message to a specified user.
bool CNDKUserManager::SendMessage(long lSockId, CNDKMessage& message)
{
    bool      bResult = false;

    m_mxUsers.Lock();

    CNDKUser  user = GetUserFromId(lSockId);

    if (user.GetId() > 0) {

        bResult = user.SendMessage(message);

        if (!bResult)
            RemoveUser(user.GetId());
    }

    m_mxUsers.Unlock();

    return bResult;
}


// Sends a message to all users.
bool CNDKUserManager::SendMessageToAllUsers(CNDKMessage& message, CLongList& listIds)
{
    m_mxUsers.Lock();

    CUserList::iterator itUser = m_users.begin();

    while (itUser != m_users.end()) {

        CNDKUser& user = *itUser++;

        if (!user.SendMessage(message))
            listIds.push_back(user.GetId());
    }

    m_mxUsers.Unlock();

    bool bResult = listIds.empty();

    CLongList::iterator itId= listIds.begin();

    while (itId != listIds.end())
        RemoveUser(*itId++);

    return bResult;
}


// Sends a message to all users except for user specified in alSockIds.
bool CNDKUserManager::SendMessageToAllUsersExceptFor(const CLongArray& alSockIds,
                                                     CNDKMessage& message,
                                                     CLongList& listIds)
{
    m_mxUsers.Lock();

    CUserList::iterator itUser = m_users.begin();

    while (itUser != m_users.end()) {

        CNDKUser& user = *itUser++;

        bool bFound = false;

        for (long lUserIndex = 0; lUserIndex < (long)alSockIds.size(); lUserIndex++) {

            if (alSockIds[lUserIndex] == user.GetId()) {
                bFound = true;
                break;
            }
        }

        // Send the message only if the user isn't in the array of exception
        if (!bFound && !user.SendMessage(message))
            listIds.push_back(user.GetId());
    }
    m_mxUsers.Unlock();

    bool bResult = listIds.empty();

    CLongList::iterator itId = listIds.begin();

    while (itId != listIds.end())
        RemoveUser(*itId++);

    return bResult;

}


// Processes pending read.
bool CNDKUserManager::ProcessPendingRead(CNDKServerSocket* pSocket, long lErrorCode, long& lSockId, CNDKMessageList& messages)
{
    bool      bResult = false;

    m_mxUsers.Lock();

    CNDKUser user = GetUserFromSocket(pSocket);

    lSockId = user.GetId();

    if ((lErrorCode == 0) && (lSockId > 0)) {

        do {

            CNDKMessage message;
            bResult = user.ReadMessage(message);
            if ( !bResult) break;
            else {
                messages.push_back(message);
            }

        } while ( !user.IsSocketBufferEmpty());
    }

    m_mxUsers.Unlock();

    return bResult;
}

////////////////////////////////////////////////////////////////////////////////
// Private Operations                                                         //
////////////////////////////////////////////////////////////////////////////////

// Gets a socket from an Id.
CNDKServerSocket* CNDKUserManager::GetSocketFromId(long lSockId) const
{
    CNDKServerSocket* lpServerSocket = NULL;

    CNDKUser user = GetUserFromId(lSockId);
    if (user.GetId() > 0) {
        lpServerSocket = user.GetSocket();
    }

    return lpServerSocket;
}

// Gets a user from an Id.
CNDKUser CNDKUserManager::GetUserFromId(long lSockId) const
{
    CNDKUser retUser;
    CUserList::const_iterator itUser = m_users.begin();

    while (itUser != m_users.end()) {

        CNDKUser user = *itUser++;

        if (user.GetId() == lSockId) {
            retUser = user;
            itUser  = m_users.end();
        }
    }

    return retUser;
}


// Gets a user from a socket.
CNDKUser CNDKUserManager::GetUserFromSocket(CNDKServerSocket* pServerSocket) const
{
    CNDKUser retUser;
    CUserList::const_iterator itUser = m_users.begin();

    while (itUser != m_users.end()) {

        CNDKUser user = *itUser++;

        if (user.IsSocketEqual(pServerSocket)) {
            retUser = user;
            itUser  = m_users.end();
        }
    }

    return retUser;
}
