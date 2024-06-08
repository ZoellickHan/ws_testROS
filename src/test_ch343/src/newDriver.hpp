// C++ system
#include <vector>
#include <functional>
#include <future>
#include <map>
#include <memory>
// #include <std_msgs/msg/float64.hpp>
// #include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

#include "protocol.hpp"
#include "crc.hpp" 

#define ROSCOMM_BUFFER_SIZE 2048
namespace newSerialDriver
{

enum class StopBit
{
    ONE,
    ONE_POINT_FIVE,
    TWO
};

enum class Parity
{
    NONE,
    ODD,
    EVEN,
    MARK,
    SPACE
};

class  SerialConfig
{  
public: 
    SerialConfig() = delete;
    SerialConfig(int bps, int databit, bool flow, StopBit stopbits, Parity paritys)
    {
        baudrate = bps;
        databits = databit;
        flowcontrol = flow;
        stopbit = stopbits;
        parity  = paritys;
    }
    ~SerialConfig();
	char* devname = (char*)"/dev/ttyCH343USB0";
    int baudrate = 2000000;
    int databits = 8;
    bool flowcontrol = 0;
    StopBit stopbit = StopBit::ONE;
    Parity  parity  = Parity::NONE;
};

class Port
{
public:
    Port(std::shared_ptr<newSerialDriver::SerialConfig> ptr);
    ~Port();

    bool init();
    int  openPort();
    bool closePort();
    bool setBaudRate();
    bool setFlowControl(bool isflowcontrol);

    bool reopen();
    int  receive();
    int  transmit(std::vector<uint8_t> & buff);
    void registerType(int typeIDArray[ID_NUM], int num);

    template <typename T>
    int decode();

    template <typename T> 
    void Classify(T& data);

    bool isPortInit();
    bool isPortOpen();
    rm_serial_driver::TwoCRC_GimbalMsg& getTwoCRC_GimbalMsg(){return twoCRC_GimbalMsg;}
    rm_serial_driver::TwoCRC_SentryGimbalMsg& getTwoCRC_SentryGimbalMsg(){return twoCRC_SentryGimbalMsg;} 

private:
    std::shared_ptr<SerialConfig> config;

    int fd;
    int num_per_read     = 0;
    int num_per_write    = 0;
    int error_count      = 0;
    int putinIndex       = 0; 
	int putoutIndex	     = 0;

    bool handshake;
    int decodeCorrectNum = 0;
    bool crc_ok_header = false;
    bool crc_ok = false;
    bool isinit = false;
    bool isopen = false;
    int interestID[ID_NUM];
    uint8_t RxBuff[ROSCOMM_BUFFER_SIZE];
    uint8_t TxBuff[ROSCOMM_BUFFER_SIZE];

    rm_serial_driver::Header header;
    std::vector<uint8_t> transform;

    rm_serial_driver::TwoCRC_GimbalMsg twoCRC_GimbalMsg;
    rm_serial_driver::TwoCRC_SentryGimbalMsg twoCRC_SentryGimbalMsg;

};

}//newSerialDriver
