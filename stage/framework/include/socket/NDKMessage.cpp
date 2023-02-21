////////////////////////////////////////////////////////////////////////////////
// Includes                                                                   //
////////////////////////////////////////////////////////////////////////////////
#include "NDKMessage.h"
#include <string.h>
#include <assert.h>

using namespace DRAFramework;

#define SAFE_DELETE_ARRAY(p)                            \
{                                                       \
    if (p) {                                            \
        delete[] p;                                     \
        p = NULL;                                       \
    }                                                   \
}

////////////////////////////////////////////////////////////////////////////////
// Constructors / Destructor                                                  //
////////////////////////////////////////////////////////////////////////////////
// Constructor.
CNDKMessage::CNDKMessage()
{
    m_lpBuffer = NULL;
    m_lLength  = 0;
}


// Constructor with untyped data.
CNDKMessage::CNDKMessage(void* lpBuffer, long lLength)
{
    m_lLength = lLength;

    if (m_lLength == 0) {
        m_lpBuffer = NULL;
    }
    else {
        m_lpBuffer = new unsigned char[m_lLength];
        memcpy(m_lpBuffer, lpBuffer, m_lLength);
    }
}


// Copy-Constructor.
CNDKMessage::CNDKMessage(const CNDKMessage& message)
{
    m_lLength = message.GetLength();

    if (m_lLength == 0) {
        m_lpBuffer = NULL;
    }
    else {
        m_lpBuffer = new unsigned char[m_lLength];
        memcpy(m_lpBuffer, message.GetBuffer(), m_lLength);
    }
}


// Destructor.
CNDKMessage::~CNDKMessage()
{
    SAFE_DELETE_ARRAY( m_lpBuffer );
}


////////////////////////////////////////////////////////////////////////////////
// Operators                                                                  //
////////////////////////////////////////////////////////////////////////////////

// Assignment operator.

const CNDKMessage& CNDKMessage::operator=(const CNDKMessage& message)
{
    if (this != &message) {

        SetBuffer(message.GetBuffer(), message.GetLength());
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
// Attributes                                                                 //
////////////////////////////////////////////////////////////////////////////////

// Sets the buffer.
void CNDKMessage::SetBuffer(void* lpBuffer, long lLength)
{
    SAFE_DELETE_ARRAY( m_lpBuffer );

    m_lLength = lLength;

    if (m_lLength == 0) {
        m_lpBuffer = NULL;
    }
    else {
        m_lpBuffer = new unsigned char[m_lLength];
        memcpy(m_lpBuffer, lpBuffer, m_lLength);
    }
}


// Gets the buffer.
unsigned char* CNDKMessage::GetBuffer() const
{
    return m_lpBuffer;
}


// Gets the length of the buffer.
unsigned int CNDKMessage::GetLength() const
{
    return m_lLength;
}
