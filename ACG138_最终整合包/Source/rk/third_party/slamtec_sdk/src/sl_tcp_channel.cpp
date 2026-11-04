/* License and attribution: LICENSES/source_notices/notice_031.txt */






 /* License and attribution: LICENSES/source_notices/notice_018.txt */
























#include "sl_lidar_driver.h"
#include "hal/abs_rxtx.h"
#include "hal/socket.h"


namespace sl {
    
    class TcpChannel : public IChannel
    {
    public:
        TcpChannel(const std::string& ip, int port) : _binded_socket(rp::net::StreamSocket::CreateSocket()) {
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
            return IS_OK(_binded_socket->connect(_socket));
            
        }

        void close()
        {
            _binded_socket->dispose();
            _binded_socket = NULL;
        }
        void flush()
        {
        
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
            return _binded_socket->send(data, size);
        }

        int read(void* buffer, size_t size)
        {
            size_t lenRec = 0;
            _binded_socket->recv(buffer, size, lenRec);
            return (int)lenRec;
        }

        void clearReadCache() {}

        void setStatus(_u32 flag){}

        int getChannelType() {
            return CHANNEL_TYPE_TCP;
        }
    private:
        rp::net::StreamSocket * _binded_socket;
        rp::net::SocketAddress _socket;
        std::string _ip;
        int _port;
    };
    Result<IChannel*> createTcpChannel(const std::string& ip, int port)
    {
        return new  TcpChannel(ip, port);
    }
}