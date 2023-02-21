////////////////////////////////////////////////////////////////////////////////
// Includes                                                                   //
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include "Poco/Net/TCPServerConnection.h"
#include "Poco/Net/TCPServerConnectionFactory.h"
#include "Poco/Net/StreamSocket.h"

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
    class CNDKServer;

    class CNDKSocket
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////

        // Constructor.
        CNDKSocket();

        // Constructor with initialization.
        CNDKSocket(Poco::Net::StreamSocket* lpStreamSocket, CNDKServer* lpServer);

        // Destructor.
        virtual ~CNDKSocket();

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
        bool SendMessage(CNDKMessage& message);

        // Receive a message.
        bool ReceiveMessage(CNDKMessage& message);

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

        //
        virtual void run();

    private:

        Poco::Net::StreamSocket*        m_lpStreamSocket;
        CNDKServer*                     m_lpServer;
        bool                            m_bIsRunning;

        friend class CNDKServerSocket;
        friend class CNDKClientSocket;
    };

    class CNDKServerConnectionFactory: public Poco::Net::TCPServerConnectionFactory
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////
        CNDKServerConnectionFactory(CNDKServer* lpServer) {
            m_lpServer = lpServer;
        };

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////

        virtual Poco::Net::TCPServerConnection*
            createConnection(const Poco::Net::StreamSocket& StreamSocket);

    private:
        CNDKServer* m_lpServer;
#ifdef __XENO__
        RT_TASK                            m_hRTTask[10];
        static int                         m_iRTTask;
#endif
    };

    class CNDKServerSocket : public Poco::Net::TCPServerConnection, public CNDKSocket
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////
        CNDKServerSocket(const Poco::Net::StreamSocket& StreamSocket, CNDKServer* lpServer)
            : Poco::Net::TCPServerConnection(StreamSocket), CNDKSocket(&socket(), lpServer) {
        };

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////

        virtual void run() {
            CNDKSocket::run();
        };

        ////////////////////////////////////////////////////////////////////////////
        // Overrides from CSocket                                                 //
        ////////////////////////////////////////////////////////////////////////////

        // Called when a new connection attempts to connect.
        void OnAccept(int nErrorCode);

        // Called when data is received.
        void OnReceive(int nErrorCode);

    };

    class CNDKClientSocket : public Poco::Runnable, public CNDKSocket
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////
        CNDKClientSocket(Poco::Net::StreamSocket* lpStreamSocket, CNDKServer* lpServer)
            : CNDKSocket(lpStreamSocket, lpServer) {
        };

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////

        virtual void run() {
            CNDKSocket::run();
        };
    };
}
