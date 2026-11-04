/* License and attribution: LICENSES/source_notices/notice_031.txt */






 /* License and attribution: LICENSES/source_notices/notice_018.txt */
























#include "sl_lidar_driver.h"
#include "hal/abs_rxtx.h"
#include "hal/socket.h"


namespace sl {
    
    class SerialPortChannel : public ISerialPortChannel
    {
    public:
        SerialPortChannel(const std::string& device, int baudrate) :_rxtxSerial(rp::hal::serial_rxtx::CreateRxTx())
        {
            _device = device;
            _baudrate = baudrate;
        }

        ~SerialPortChannel()
        {
            if (_rxtxSerial)
                delete _rxtxSerial;
        }

        bool bind(const std::string& device, sl_s32 baudrate)
        {
            _closePending = false;
            return _rxtxSerial->bind(device.c_str(), baudrate);
        }

        bool open()
        {
            if(!bind(_device, _baudrate))
                return false;
            return _rxtxSerial->open();
        }

        void close()
        {
            _closePending = true;
            _rxtxSerial->cancelOperation();
            _rxtxSerial->close();
        }
        void flush()
        {
            _rxtxSerial->flush(0);
        }

        sl_result waitForDataExt(size_t& size_hint, sl_u32 timeoutInMs)
        {
            _word_size_t result;
            size_t size_holder;
            size_hint = 0;

            if (_closePending) return  RESULT_OPERATION_TIMEOUT;

            if (!_rxtxSerial->isOpened()) {
                return RESULT_OPERATION_FAIL;
            }

            result = _rxtxSerial->waitfordata(1, timeoutInMs, &size_holder);
            size_hint = size_holder;
            if (result == (_word_size_t)rp::hal::serial_rxtx::ANS_DEV_ERR)
                return RESULT_OPERATION_FAIL;
            if (result == (_word_size_t)rp::hal::serial_rxtx::ANS_TIMEOUT)
                return RESULT_OPERATION_TIMEOUT;

            return RESULT_OK;
        }

        bool waitForData(size_t size, sl_u32 timeoutInMs, size_t* actualReady)
        {
            if (_closePending) return false;
            return (_rxtxSerial->waitfordata(size, timeoutInMs, actualReady) == rp::hal::serial_rxtx::ANS_OK);
        }

        int write(const void* data, size_t size)
        {
           return _rxtxSerial->senddata((const sl_u8 * )data, size);
        }

        int read(void* buffer, size_t size)
        {
            size_t lenRec = 0;
            lenRec = _rxtxSerial->recvdata((sl_u8 *)buffer, size);
            return (int)lenRec;
        }

        void clearReadCache()
        {
           
        }

        void setDTR(bool dtr)
        {
            dtr ? _rxtxSerial->setDTR() : _rxtxSerial->clearDTR();
        }

        int getChannelType() {
            return CHANNEL_TYPE_SERIALPORT;
        }

    private:
        rp::hal::serial_rxtx  * _rxtxSerial;
        bool _closePending;
        std::string _device;
        int _baudrate;

    };

    Result<IChannel*> createSerialPortChannel(const std::string& device, int baudrate)
    {
        return new  SerialPortChannel(device, baudrate);
    }

}