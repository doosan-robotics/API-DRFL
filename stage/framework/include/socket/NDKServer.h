////////////////////////////////////////////////////////////////////////////////
// Includes                                                                   //
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include "Poco/Net/ServerSocket.h"
#include "Poco/Net/TCPServer.h"

#include "NDKUserManager.h"

namespace DRAFramework
{
    ////////////////////////////////////////////////////////////////////////////////
    // Forward declarations                                                       //
    ////////////////////////////////////////////////////////////////////////////////
    class CNDKMessage;
    class CNDKServerConnectionFactory;

    ////////////////////////////////////////////////////////////////////////////////
    // Defines                                                                    //
    ////////////////////////////////////////////////////////////////////////////////

    // Enumeration of all type of disconnection
    enum NDKServerDisconnection
    {
        NDKServer_NormalDisconnection,
        NDKServer_ClientCloseConnection,
        NDKServer_ErrorSendingMessage,
        NDKServer_ErrorReceivingMessage
    };

    class TCPServerEx : public Poco::Net::TCPServer
    {

    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////

        // Constructor.
        TCPServerEx(Poco::Net::TCPServerConnectionFactory::Ptr pFactory, const Poco::Net::ServerSocket& socket);

        // Destructor.
        virtual ~TCPServerEx();

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////
        virtual void run();

    private:
#ifdef __XENO__
        RT_TASK                            m_hRTTask;
        static int                         m_iRTTask;
#endif
    };

    class CNDKServer
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////

        // Constructor.
        CNDKServer();

        // Destructor.
        virtual ~CNDKServer();

        ////////////////////////////////////////////////////////////////////////////
        // Attributes                                                             //
        ////////////////////////////////////////////////////////////////////////////

        // Returns if the server is started.
        bool IsStarted() const;

        // Returns the port.
        int GetPort() const;

        // Returns the number of connected users.
        long GetNbUsers() const;

        // Returns the Ids of all connected users.
        void GetUserIds(CLongArray& alIds);

        // Gets a socket from an Id.
        CNDKServerSocket* GetUserSocket(long lUserId);
        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////

        // Starts listening on the specified port.
        bool StartListening(int iPort);

        // Stops the server.
        void Stop();

        // Sends a message to a specified user. If a problem occurs, OnDisconnect
        // callback will be called.
        bool SendMessageToUser(long lUserId, CNDKMessage& message);

        // Sends a message to all users. OnDisconnect callback will be called for
        // each user that the message cannot be sent.
        bool SendMessageToAllUsers(CNDKMessage& message);

        // Sends a message to some user specified by the array of user Id.
        // OnDisconnect callback will be called for each user that the message
        // cannot be sent.
        bool SendMessageToSomeUsers(const CLongArray& alUserIds, CNDKMessage& message);

        // Sends a message to all users except for the specified user Id.
        // OnDisconnect callback will be called for each user that the message
        // cannot be sent.
        bool SendMessageToAllUsersExceptFor(long lUserId, CNDKMessage& message);

        // Sends a message to all users that aren't in the specified array of user
        // Id. OnDisconnect callback will be called for each user that the message
        // cannot be sent.
        bool SendMessageToAllUsersExceptFor(const CLongArray& alUserIds, CNDKMessage& message);

        // Disconnects a specified user. OnDisconnect callback will not be called
        // for user disconnected that way.
        bool DisconnectUser(long lUserId);

        // Disconnects all users. OnDisconnection callback will not be call.
        void DisconnectAllUsers();

        ////////////////////////////////////////////////////////////////////////////
        // Callbacks                                                              //
        ////////////////////////////////////////////////////////////////////////////
        // Called when a user tries to connect to the server. Return TRUE to accept
        // the connection or FALSE otherwise. The derived class must override this
        // method.
        virtual bool OnIsConnectionAccepted() = 0;

        // Called when a user is connected to the server. The derived class must
        // override this method.
        virtual void OnConnect(long lUserId) = 0;

        // Called whenever a message is received from a user. The derived class must
        // override this method.
        virtual void OnMessage(long lUserId, CNDKMessage& message) = 0;

        // Called whenever a user is disconnected (the user might have closed
        // the connection or an error occurred when sending a message, for example).
        // DisconnectUser don't need to be called when OnDisconnect callback is
        // used. The derived class must override this method.
        virtual void OnDisconnect(long lUserId, NDKServerDisconnection disconnectionType) = 0;

    private:
        ////////////////////////////////////////////////////////////////////////////
        // Private Operations                                                     //
        ////////////////////////////////////////////////////////////////////////////

        // Processes pending accept.
        void ProcessPendingAccept(CNDKServerSocket* pServerSocket, long lErrorCode);

        // Processes pending read.
        bool ProcessPendingRead(CNDKServerSocket* pServerSocket, long lErrorCode);

        // Translates message and does the appropriate task for message handled by
        // the NDK.
        void TranslateMessage(long lUserId, CNDKMessage& message);

        // Disconnects a user.
        bool DisconnectUser(long lUserId, NDKServerDisconnection disconnectionType);

        ////////////////////////////////////////////////////////////////////////////
        // Friendship                                                             //
        ////////////////////////////////////////////////////////////////////////////

        friend class CNDKServerSocket;

        ////////////////////////////////////////////////////////////////////////////
        // Disable Copy-Constructor and Assignment Operator                       //
        ////////////////////////////////////////////////////////////////////////////

        CNDKServer(const CNDKServer&);
        void operator=(const CNDKServer &);

protected:

        Poco::Net::ServerSocket*            m_lpSeverSocket;
        int                                 m_iPort;

        TCPServerEx*                        m_lpServer;
       
        CNDKUserManager                     m_mgrCtrlIds;
    };
}
