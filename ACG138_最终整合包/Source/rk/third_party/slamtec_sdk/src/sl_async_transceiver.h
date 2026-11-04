/* License and attribution: LICENSES/source_notices/notice_025.txt */






 /* License and attribution: LICENSES/source_notices/notice_018.txt */
























#pragma once

#include <list>
#include <memory>

namespace sl { namespace internal {


class _single_thread ProtocolMessage {

public:
	size_t len;			
	_u8 cmd;
protected:
	_u8* data;
	size_t _databufsize;

public:
	ProtocolMessage();
	ProtocolMessage(_u8 cmd, const void* buffer, size_t size);
	ProtocolMessage(const ProtocolMessage& srcMsg);
	virtual ~ProtocolMessage();

	ProtocolMessage& operator=(const ProtocolMessage& srcMessage);

	// avoid use this method, pls. use fillData instead
	void setDataBuf(_u8* buffer, size_t size);

	_u8* getDataBuf() { return data; }

	void fillData(const void* buffer, size_t size);
	void cleanData();

	size_t getPayloadSize() const
	{
		return len;
	}

protected:

	// change the data buffer to fix the new payload size
	// the existing buffer will be reused if possible.
	// all the existing payload data will lose
	void _changeBufSize(bool force_compact = false);
	bool _usingOutterData;
};



typedef std::shared_ptr<ProtocolMessage> message_autoptr_t;


class IAsyncProtocolCodec {
public:
	IAsyncProtocolCodec() {}
	virtual ~IAsyncProtocolCodec()  {}

	virtual void   onChannelError(u_result errCode) {}

	virtual void   onDecodeReset() {}
	virtual void   onDecodeData(const void* buffer, size_t size) = 0;


	virtual size_t estimateLength(message_autoptr_t& message) = 0;
	virtual void   onEncodeData(message_autoptr_t& message, _u8* txbuffer, size_t* size) = 0;

};

class AsyncTransceiver {
public:

	enum working_flag_t
	{
		WORKING_FLAG_RX_DISABLED = 0x1L << 0,
		WORKING_FLAG_TX_DISABLED = 0x1L << 1,

		WORKING_FLAG_ERROR = 0x1L << 31,
	};


	AsyncTransceiver(IAsyncProtocolCodec& codec);
	~AsyncTransceiver();



	u_result openChannelAndBind(IChannel* channel);
	void     unbindAndClose();

	IChannel* getBindedChannel() const {
		return _bindedChannel;
	}
	
	u_result sendMessage(message_autoptr_t& msg);

protected:

	sl_result _proc_rxThread();
	sl_result _proc_decoderThread();

protected:


	rp::hal::Locker _opLocker;
	rp::hal::Locker _rxLocker;
	rp::hal::Event  _dataEvt;

	IChannel* _bindedChannel;
	IAsyncProtocolCodec& _codec;


	bool _isWorking;
	_u32 _workingFlag;

	rp::hal::Thread _rxThread;
	rp::hal::Thread _decoderThread;

	struct Buffer {
		size_t size;
		_u8* data;


		Buffer() : size(0), data(NULL){}

		~Buffer() {
			if (data) {
				delete[] data;
				data = NULL;
			}
		}
	};
	std::list< Buffer * > _rxQueue;
};


}}
