/* License and attribution: LICENSES/source_notices/notice_031.txt */






 /* License and attribution: LICENSES/source_notices/notice_018.txt */
























#include "sl_lidar_driver.h"
#include "hal/abs_rxtx.h"
#include "hal/socket.h"


namespace sl {
	class UdpChannel : public IChannel
	{
	public:
		UdpChannel(const std::string& ip, int port) : _binded_socket(rp::net::DGramSocket::CreateSocket()) {
            _ip = ip;
            _port = port;
        }

		bool bind(const std::string & ip, sl_s32 port)
        {
            _socket = rp::net::SocketAddress(ip.c_str(), port);
            return true;
        }

        bool open()
        {
            if(!bind(_ip, _port))
                return false;
            return SL_IS_OK(_binded_socket->setPairAddress(&_socket));         
        }

        void close()
        {
            _binded_socket->dispose();
            _binded_socket = NULL;
        }
        void flush()
        {
            clearReadCache();
        }

        sl_result waitForDataExt(size_t& size_hint, sl_u32 timeoutInMs)
        {
            u_result ans;
            size_hint = 0;
            ans = _binded_socket->waitforData(timeoutInMs);

            switch (ans) {
            case RESULT_OK:
                size_hint = 1024; //dummy value
                break;
            }

            return ans;
        }

        bool waitForData(size_t size, sl_u32 timeoutInMs, size_t* actualReady)
        {
            if (actualReady)
                *actualReady = size;
            return (_binded_socket->waitforData(timeoutInMs) == RESULT_OK);

        }

        int write(const void* data, size_t size)
        {
            return _binded_socket->sendTo(nullptr, data, size);
        }

        int read(void* buffer, size_t size)
        {
            size_t actualGet;

            u_result ans = _binded_socket->recvFrom(buffer, size, actualGet);
            if (IS_FAIL(ans)) return 0;
            return actualGet;
        
        }

        void clearReadCache() {
            _binded_socket->clearRxCache();
        }

        void setStatus(_u32 flag){}
        
        int getChannelType() {
            return CHANNEL_TYPE_UDP;
        }

	private:
		rp::net::DGramSocket * _binded_socket;
		rp::net::SocketAddress _socket;
        std::string _ip;
        int _port;
	};

    Result<IChannel*> createUdpChannel(const std::string& ip, int port)
    {
        return new  UdpChannel(ip, port);
    }
}