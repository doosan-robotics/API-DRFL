#pragma once

namespace DRAFramework {

    class CNDKMessage
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////

        // Constructor.
        CNDKMessage();

        // Constructor with untyped data.
        CNDKMessage(void* lpBuffer, long lLength);

        // Copy-Constructor.
        CNDKMessage(const CNDKMessage& message);

        // Destructor.
        virtual ~CNDKMessage();
        ////////////////////////////////////////////////////////////////////////////
        // Operators                                                              //
        ////////////////////////////////////////////////////////////////////////////

        // Assignment operator.
        const CNDKMessage& operator=(const CNDKMessage& message);

        ////////////////////////////////////////////////////////////////////////////
        // Attributes                                                             //
        ////////////////////////////////////////////////////////////////////////////

        // Sets the buffer.
        void SetBuffer(void* lpBuffer, long lLength);

        // Gets the buffer.
        unsigned char* GetBuffer() const;

        // Gets the length of the buffer.
        unsigned int GetLength() const;

    private:
        unsigned char*     m_lpBuffer;
        long               m_lLength;
    };
}
