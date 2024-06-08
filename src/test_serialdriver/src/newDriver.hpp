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

// C system

#define USE_CH343_DRIVER 1
#if USE_CH343_DRIVER
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
    //TEST
    void test_receive();
    int test_transmit();
    long getNumRead(){return num_read;}
    long getNumWrite(){return num_write;}
    int getErrorCount(){return error_count;}

    bool reopen();
    bool init();
    bool isPortInit();
    int  openPort();
    bool isPortOpen();
    bool closePort();
    int  receive(std::vector<uint8_t> & buff);
    int  transmit(std::vector<uint8_t> & buff);
    bool setBaudRate();
    bool setFlowControl(bool isflowcontrol);

private:
    std::shared_ptr<SerialConfig> config;

    int fd;
    int num_per_read     =0;
    int error_count      =0;
    int num_per_write    =0;
	long num_read        =0;
    long num_write       =0;

    bool handshake;
    bool isinit = false;
    bool isopen = false;
    uint8_t RxBuff[2048];
    uint8_t TxBuff[2048];
};

}//newSerialDriver
#endif