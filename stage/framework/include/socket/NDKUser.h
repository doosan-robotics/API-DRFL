////////////////////////////////////////////////////////////////////////////////
// Includes                                                                   //
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include "NDKMessage.h"

#ifndef __XENO__
#undef SendMessage
#endif

namespace DRAFramework {
    ////////////////////////////////////////////////////////////////////////////////
    // Forward declarations                                                       //
    ////////////////////////////////////////////////////////////////////////////////
    class CNDKServerSocket;

    class CNDKUser
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////

        // Constructor.
        CNDKUser();

        // Constructor with initialization.
        CNDKUser(long lId, CNDKServerSocket* pServerSocket);

        // Destructor.
        virtual ~CNDKUser();

        ////////////////////////////////////////////////////////////////////////////
        // Attributes                                                             //
        ////////////////////////////////////////////////////////////////////////////

        // Gets the Id.
        long GetId() const;

        // Returns if the socket is the same as specified.
        bool IsSocketEqual(CNDKServerSocket* pServerSocket) const;

        // Returns if the buffer of the socket is empty.
        bool IsSocketBufferEmpty() const;

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////

        // Sends a message.
        bool SendMessage(CNDKMessage& message);

        // Reads a message.
        bool ReadMessage(CNDKMessage& message);

        // Closes the socket.
        void CloseSocket();

        CNDKServerSocket* GetSocket() { return m_lpServerSocket; }

        ////////////////////////////////////////////////////////////////////////////
        // Operators                                                              //
        ////////////////////////////////////////////////////////////////////////////

        // Comparison operator.
        bool operator==(const CNDKUser& user) const;

    private:
        long              m_lId;
        CNDKServerSocket* m_lpServerSocket;
    };
}
