// C++ system
#include <vector>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <atomic>

#include "protocol.hpp"
#include "crc.hpp" 

#define ROSCOMM_BUFFER_SIZE 2048
#define BUFFER_SIZE 512

namespace rm_serial_driver
{
namespace newSerialDriver
{

class  SerialConfig
{  
public: 
    enum StopBit : uint8_t
    {
        ONE,
        ONE_POINT_FIVE,
        TWO
    };

    enum Parity : uint8_t
    {
        NONE,
        ODD,
        EVEN,
        MARK,
        SPACE
    };

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

enum CRCstate : uint8_t
{
    CRC_OK = 0,
    CRC_ERROR_HEADER = 1,
    CRC_ERROR_DATA = 2,
};

    Port(std::shared_ptr<newSerialDriver::SerialConfig> ptr);
    ~Port();

    bool init();
    int  openPort();
    bool closePort();
    bool reopen();

    int  transmit(std::vector<uint8_t> & buff);   

    //VERSIOM THREE
    void  receive();
    void  putinIndexHandle(int size);
    void  jumpHandle(int length1, int length2);
    void  putoutIndexHandle();
    bool  decodeHandle(int ID);
    // void  testingMCU();
    
    bool isPortInit();
    bool isPortOpen();
    long& getsum(){return sum;}
    int& geterrorHeader(){return error_header_count ;}
    int& geterrorData(){return error_data_count;}
    int& getdecodeCount(){return decodeCount;}
    int& getputinIndex(){return putinIndex;}
    int& getputoutIndex(){return putoutIndex;}

    CRCstate& CRCnotice(){return currentCRC;}

    std::thread receiveThread;
    std::thread decodeThread;

    rm_serial_driver::TwoCRC_GimbalMsg& getTwoCRC_GimbalMsg(){return twoCRC_GimbalMsg;}
    rm_serial_driver::TwoCRC_SentryGimbalMsg& getTwoCRC_SentryGimbalMsg(){return twoCRC_SentryGimbalMsg;} 
    rm_serial_driver::GimbalMsg& getGimbalMsg(){return gimbalMsg;}
    rm_serial_driver::SentryGimbalMsg& getSentryGimbalMsg(){return sentryGimbalMsg;}

private:

    std::shared_ptr<SerialConfig> config;
    int fd;
    long sum                = 0;
    int num_per_read        = 0;
    int num_per_write       = 0;
    int error_header_count  = 0;
    int error_data_count    = 0;
    int decodeCount         = 0;
    int putinIndex          = 0;
    int putoutIndex         = 0;
    bool ifdecode           = false;
    bool crc_ok_header      = false;
    bool crc_ok             = false;
    bool isinit             = false;
    bool isopen             = false;   

    uint8_t ReadBuffer[BUFFER_SIZE];
    uint8_t decodeBuffer[BUFFER_SIZE];
    uint8_t RxBuff[ROSCOMM_BUFFER_SIZE];
    uint8_t TxBuff[ROSCOMM_BUFFER_SIZE];
    uint8_t jumpBuff[ROSCOMM_BUFFER_SIZE];
    

    CRCstate currentCRC = CRCstate::CRC_OK;
    std::vector<uint8_t> transform;
    rm_serial_driver::Header header;
    rm_serial_driver::TwoCRC_GimbalMsg twoCRC_GimbalMsg;
    rm_serial_driver::TwoCRC_SentryGimbalMsg twoCRC_SentryGimbalMsg;
    rm_serial_driver::GimbalMsg gimbalMsg;
    rm_serial_driver::SentryGimbalMsg sentryGimbalMsg;
};

}//newSerialDriver
}//rm_serial_driver