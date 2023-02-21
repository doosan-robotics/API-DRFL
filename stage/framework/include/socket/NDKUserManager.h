////////////////////////////////////////////////////////////////////////////////
// Includes                                                                   //
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include "NDKMessage.h"
#include "NDKUser.h"
#include "../SyncInterface.h"
#include <list>
#include <vector>

#ifndef __XENO__
#undef SendMessage
#endif

namespace DRAFramework {

    ////////////////////////////////////////////////////////////////////////////////
    // Forward declarations                                                       //
    ////////////////////////////////////////////////////////////////////////////////
    class CNDKServerSocket;

    ////////////////////////////////////////////////////////////////////////////////
    // Defines                                                                    //
    ////////////////////////////////////////////////////////////////////////////////
    typedef std::vector<long>        CLongArray;
    typedef std::list<CNDKUser>      CUserList;
    typedef std::list<long>          CLongList;
    typedef std::list<CNDKMessage>   CNDKMessageList;

    class CNDKUserManager
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////

        // Constructor.
        CNDKUserManager();

        // Destructor.
        virtual ~CNDKUserManager();

        ////////////////////////////////////////////////////////////////////////////
        // Attributes                                                             //
        ////////////////////////////////////////////////////////////////////////////

        // Gets the number of users.
        long GetNbUsers() const;

        // Returns the Ids of all users.
        void GetUserIds(CLongArray& alIds) const;

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////

        // Adds a user then returns its new Id.
        long AddUser(CNDKServerSocket* pServerSocket);

        // Removes a user.
        bool RemoveUser(long lSockId);

        // Removes all users.
        void RemoveAllUsers();

        // Sends a message to a specified user.
        bool SendMessage(long lSockId, CNDKMessage& message);

        // Sends a message to all users.
        bool SendMessageToAllUsers(CNDKMessage& message, CLongList& listIds);

        // Sends a message to all users except for user specified in alSockIds.
        bool SendMessageToAllUsersExceptFor(const CLongArray& alSockIds, CNDKMessage& message, CLongList& listIds);

        // Processes pending read.
        bool ProcessPendingRead(CNDKServerSocket* pSocket, long lErrorCode, long& lSockId, CNDKMessageList& messages);

    private:
        ////////////////////////////////////////////////////////////////////////////
        // Private Operations                                                     //
        ////////////////////////////////////////////////////////////////////////////

        // Gets a socket from an Id.
        CNDKServerSocket* GetSocketFromId(long lSockId) const;

        // Gets a user from an Id.
        CNDKUser GetUserFromId(long lSockId) const;

        // Gets a user from a socket.
        CNDKUser GetUserFromSocket(CNDKServerSocket*) const;

    private:
        CUserList                   m_users;
        long                        m_lNextId;

        mutable CMutex              m_mxUsers;
        friend class                CNDKServer;
    };
}
