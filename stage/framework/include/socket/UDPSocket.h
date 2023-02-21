////////////////////////////////////////////////////////////////////////////////
// Includes                                                                   //
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/SocketAddress.h>
#include <Poco/Thread.h>
#include <Poco/Runnable.h>
#include <Poco/Event.h>
#include <map>
#include <list>
#include <vector>

#include "NDKMessage.h"

#ifdef __XENO__
#include <native/task.h>
#else
#undef SendMessage
#endif

namespace DRAFramework {

    ////////////////////////////////////////////////////////////////////////////////
    // Forward declarations                                                       //
    ////////////////////////////////////////////////////////////////////////////////
    class CNDKServerUDP;

    ////////////////////////////////////////////////////////////////////////////////
    // Defines                                                                    //
    ////////////////////////////////////////////////////////////////////////////////


    class CNDKSocketUDP
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////

        // Constructor.
        //CNDKSocketUDP();

        // Constructor with initialization.
        CNDKSocketUDP(CNDKServerUDP* lpServer);

        // Destructor.
        virtual ~CNDKSocketUDP();

        enum {
            READ_BUFFER_SIZE = 4096,
        };
        ////////////////////////////////////////////////////////////////////////////
        // Attributes                                                             //
        ////////////////////////////////////////////////////////////////////////////

        // Return if the buffer is empty.
        bool IsBufferEmpty() const;

        // Return IP address of remote client
        void GetPeerAddress(std::string& rIpAddress, unsigned int& rPort);

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////

        // Initializes the socket.
        bool Initialize();

        // Closes the socket.
        void Close();

        // Sends a message.
        bool SendMessage(Poco::Net::SocketAddress& rAddress, CNDKMessage& rMessage);

        // Receive a message.
        bool ReceiveMessage(Poco::Net::SocketAddress& rAddress, CNDKMessage& rMessage);

        // Shutdowns the socke
        void ShutDown();

    protected:
        ////////////////////////////////////////////////////////////////////////////
        // Overrides Operations                                                   //
        ////////////////////////////////////////////////////////////////////////////

        virtual void OnSend(int nErrorCode) { };
        virtual void OnReceive(int nErrorCode) { };
        virtual void OnAccept(int nErrorCode) { };
        virtual void OnClose(int nErrorCode) { };
        virtual void OnConnect(int nErrorCode) { };

    private:
        ////////////////////////////////////////////////////////////////////////////
        // Private Operations                                                     //
        ////////////////////////////////////////////////////////////////////////////

        // run
        void run();

    private:

        Poco::Net::DatagramSocket*          m_lpDatagramSocket;
        CNDKServerUDP*                      m_lpServer;
        bool                                m_bStop;
        Poco::Event                         m_evTask;
        friend class CNDKServerSocketUDP;
    };


    class CNDKServerSocketUDP : public CNDKSocketUDP, public Poco::Runnable
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////

        // Constructor.
        CNDKServerSocketUDP(CNDKServerUDP* lpServer);

        // Destructor.
        virtual ~CNDKServerSocketUDP();

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////

        // Starts listening on the specified port.
        bool Start(int iPort);

        // Stops the server.
        void Stop();

        ////////////////////////////////////////////////////////////////////////////
        // Overrides from CSocket                                                 //
        ////////////////////////////////////////////////////////////////////////////

        virtual void run();

        // Called when data is received.
        void OnReceive(int nErrorCode);

    private:
        Poco::Thread                            m_hTask;
#ifdef __XENO__
        RT_TASK                                 m_hRTTask;
        static int                              m_iRTTask;
#endif
    };

    class CNDKServerUDP
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////

        // Constructor.
        CNDKServerUDP();

        // Destructor.
        virtual ~CNDKServerUDP();
        typedef std::list<long>                             LISTSOCKIDS;
        typedef std::list<CNDKMessage>                      LSTMESSAGES;
        typedef std::map<long, Poco::Net::SocketAddress>    MAPSOCKMANAGER;
        typedef std::map<long, LSTMESSAGES>                 MAPSOCKMESSAGE;

        ////////////////////////////////////////////////////////////////////////////
        // Attributes                                                             //
        ////////////////////////////////////////////////////////////////////////////

        // Returns if the server is started.
        bool IsStarted() const;

        // Returns the port.
        int GetPort() const;

        // Returns the number of connected users.
        long GetNbUsers() const;

        // Gets a socket from an Id.
        void GetPeerAddress(long lUserId, std::string& rIpAddress, unsigned int& rPort);

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////

        // Starts listening on the specified port.
        bool StartListening(int iPort);

        // Stops the server.
        void Stop();

        // Sends a message to a specified user. If a problem occurs, OnDisconnect
        // callback will be called.
        bool SendMessageToUser(long lUserId, CNDKMessage& rMessage);

        // Sends a message to all users. OnDisconnect callback will be called for
        // each user that the message cannot be sent.
        bool SendMessageToAllUsers(CNDKMessage& rMessage);
        ////////////////////////////////////////////////////////////////////////////
        // Callbacks                                                              //
        ////////////////////////////////////////////////////////////////////////////

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
        virtual void OnDisconnect(long lUserId) = 0;
    private:
        ////////////////////////////////////////////////////////////////////////////
        // Private Operations                                                     //
        ////////////////////////////////////////////////////////////////////////////

        // Processes pending read.
        bool ProcessPendingRead(CNDKServerSocketUDP* pServerSocket, long lErrorCode);

        // Translates message and does the appropriate task for message handled by
        // the NDK.
        void TranslateMessage(long lUserId, CNDKMessage& message);

        ////////////////////////////////////////////////////////////////////////////
        // Friendship                                                             //
        ////////////////////////////////////////////////////////////////////////////

        friend class CNDKServerSocketUDP;

        ////////////////////////////////////////////////////////////////////////////
        // Disable Copy-Constructor and Assignment Operator                       //
        ////////////////////////////////////////////////////////////////////////////

        CNDKServerUDP(const CNDKServerUDP&);
        void operator=(const CNDKServerUDP &);

    protected:
        CNDKServerSocketUDP*                m_lpServer;
        int                                 m_iPort;
        MAPSOCKMANAGER                      m_mgrCtrlIds;
        long                                m_lNextId;
    };
}
