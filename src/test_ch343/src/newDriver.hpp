// C++ system
#include <vector>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

#include "protocol.hpp"
#include "crc.hpp" 

#define ROSCOMM_BUFFER_SIZE 1024
#define BUFFER_SIZE 512
namespace newSerialDriver
{
class  SerialConfig
{  

enum StopBit : uint8_t
{
    ONE = 0,
    ONE_POINT_FIVE,
    TWO
};

enum Parity : uint8_t
{
    NONE = 0,
    ODD,
    EVEN,
    MARK,
    SPACE
};

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

enum PkgState : uint8_t
{
    COMPLETE = 0,
    HEADER_INCOMPLETE,
    PAYLOAD_INCOMPLETE,
    CRC_HEADER_ERRROR,
    CRC_PKG_ERROR,
};

public:

    Port(std::shared_ptr<newSerialDriver::SerialConfig> ptr);
    ~Port();

    // port function 
    bool init();
    int  openPort();
    bool closePort();
    bool reopen();
    bool isPortInit();
    bool isPortOpen();

    // rx tx function
    int  transmit(std::vector<uint8_t> & buff); 
    void decodeFun();  
    void readFun();
    void putinIndexFun();
    PkgState putoutIndexFun();
    
    // data and debug function
    int& geterrorHeader(){return error_header_count;}
    int& geterrorData(){return error_data_count;}
    int& getdecodeCount(){return decodeCount;}
    rm_serial_driver::TwoCRC_GimbalMsg& getTwoCRC_GimbalMsg(){return twoCRC_GimbalMsg;}
    rm_serial_driver::TwoCRC_SentryGimbalMsg& getTwoCRC_SentryGimbalMsg(){return twoCRC_SentryGimbalMsg;} 

private:

    // port data
    std::shared_ptr<SerialConfig> config;
    int fd;
    int num_per_read        = 0;
    int num_per_write       = 0;

    bool isinit          = false;
    bool isopen          = false;

    // state data
    int rxsize              = 0;
    int putinIndex          = 0;
    int putoutIndex         = 0;
    int decodeCount         = 0;
    int error_header_count  = 0;
    int error_data_count    = 0;

    bool crc_ok_header   = false;
    bool crc_ok          = false;

    //rx tx data
    std::vector<uint8_t> transformVector;
    std::vector<uint8_t> decodeVector;

    uint8_t readBuffer[BUFFER_SIZE];
    uint8_t RxBuff[ROSCOMM_BUFFER_SIZE];
    uint8_t TxBuff[ROSCOMM_BUFFER_SIZE];

    rm_serial_driver::Header header;
    rm_serial_driver::TwoCRC_GimbalMsg twoCRC_GimbalMsg;
    rm_serial_driver::TwoCRC_SentryGimbalMsg twoCRC_SentryGimbalMsg;
};

}//newSerialDriver
