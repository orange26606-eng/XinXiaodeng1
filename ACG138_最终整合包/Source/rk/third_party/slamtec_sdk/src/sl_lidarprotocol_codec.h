/* License and attribution: LICENSES/source_notices/notice_025.txt */






 /* License and attribution: LICENSES/source_notices/notice_018.txt */

























#pragma once

#include "sl_async_transceiver.h"

namespace sl { namespace internal {


class IProtocolMessageListener {
public:
    virtual void onProtocolMessageDecoded(const ProtocolMessage&) = 0;
};


class RPLidarProtocolCodec : public IAsyncProtocolCodec
{
public:

    enum {
        STATUS_WAIT_SYNC1 = 0x0,
        STATUS_WAIT_SYNC2 = 0x1,
        STATUS_WAIT_SIZE_FLAG = 0x2,
        STATUS_WAIT_TYPE = 0x3,
        STATUS_RECV_PAYLOAD = 0x4,
        STATUS_LOOP_MODE_FLAG = 0x80000000,
    };

    RPLidarProtocolCodec();

    void exitLoopMode();


    virtual size_t estimateLength(message_autoptr_t& message);


    virtual void onEncodeData(message_autoptr_t& message, _u8* txbuffer, size_t* size);

    virtual void   onDecodeReset();
    virtual void   onDecodeData(const void* buffer, size_t size);
    
    void setMessageListener(IProtocolMessageListener* l);

protected:

    IProtocolMessageListener* _listener;
    ProtocolMessage          _decodingMessage;
    rp::hal::Locker          _op_locker;
                            
    _u32                     _working_states;
    int                      _rx_pos;
};

}}



