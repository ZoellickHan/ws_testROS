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

#include "tty.h"
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pnp.h>
#include <linux/property.h>
#include <linux/serial_core.h>
#include <linux/spinlock.h>

#include "serial_base.h"

#define ROSCOMM_BUFFER_SIZE 2048
#define BUFFER_SIZE 2048
namespace newSerialDriver
{
class  SerialConfig
{  
public: 

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

    enum PkgState : uint8_t
    {
        COMPLETE = 0,
        HEADER_INCOMPLETE,
        PAYLOAD_INCOMPLETE,
        CRC_HEADER_ERRROR,
        CRC_PKG_ERROR,
    };

    Port(std::shared_ptr<newSerialDriver::SerialConfig> ptr);
    ~Port();

    // port function 
    bool init();
    int  openPort();
    bool closePort();
    bool reopen();
    bool isPortInit();
    bool isPortOpen();

    int specialread();

    // rx tx function
    int  transmit(std::vector<uint8_t> & buff);  
    void readFun();
    void decodeFun(int ID);
    void decodeThreadFun();
    void putinIndexFun(int size);
    PkgState putoutIndexFun();

    
    // data and debug function
    int& geterrorHeader(){return error_header_count;}
    int& geterrorData(){return error_data_count;}
    int& getdecodeCount(){return decodeCount;}
    int& getputinIndex(){return putinIndex;}
    int& getputoutIndex(){return putoutIndex;}
    rm_serial_driver::TwoCRC_GimbalMsg& getTwoCRC_GimbalMsg(){return twoCRC_GimbalMsg;}
    rm_serial_driver::TwoCRC_SentryGimbalMsg& getTwoCRC_SentryGimbalMsg(){return twoCRC_SentryGimbalMsg;} 

    std::thread  readThread; 
    std::thread  decodeThread;

private:

    // port data
    std::shared_ptr<SerialConfig> config;
    int fd;
    int num_per_read        = 0;
    int num_per_write       = 0;

    bool isinit          = false;
    bool isopen          = false;

    //rx thread
    PkgState            frameState;
    // state data
    int rxsize              = 0;
    int putinIndex          = -1;
    int putoutIndex         = -1;
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
    uint8_t frameBuffer[BUFFER_SIZE];

    rm_serial_driver::Header header;
    rm_serial_driver::TwoCRC_GimbalMsg twoCRC_GimbalMsg;
    rm_serial_driver::TwoCRC_SentryGimbalMsg twoCRC_SentryGimbalMsg;
};

class ttyPort
{
    public:
    private:

        struct uart_driver;


};

uart_register_driver()

// 
//为uart_driver->state, 每个端口需要一个state
drv->state = kzalloc(sizeof(struct uart_state) * drv->nr, GFP_KERNEL);
if (!drv->state)
	goto out;
//创建一个tty_driver数据结构，并为各个端口分配资源
normal = alloc_tty_driver(drv->nr);
if (!normal)
	goto out_kfree;
//uart_driver中的tty_driver指向新创建的tty_driver，方便其他代码根据uart_driver查找tty_driver 
drv->tty_driver = normal;
//一系统的参数初始化
normal->driver_name	= drv->driver_name;
normal->name		= drv->dev_name;
normal->major		= drv->major;
normal->minor_start	= drv->minor;
normal->type		= TTY_DRIVER_TYPE_SERIAL;
normal->subtype		= SERIAL_TYPE_NORMAL;
normal->init_termios	= tty_std_termios;
normal->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
normal->init_termios.c_ispeed = normal->init_termios.c_ospeed = 9600;
normal->flags		= TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
normal->driver_state    = drv;
//设置tty_driver的ops，uart_ops由kernel核心代码提供
tty_set_operations(normal, &uart_ops);

/*
 * Initialise the UART state(s).
 */
for (i = 0; i < drv->nr; i++) {
	struct uart_state *state = drv->state + i;
	struct tty_port *port = &state->port;
	//初始化每个tty端口，核心部分是tty_buffer_init
	tty_port_init(port);
	port->ops = &uart_port_ops;
	port->close_delay     = HZ / 2;	/* .5 seconds */
	port->closing_wait    = 30 * HZ;/* 30 seconds */
}
//注册tty_driver
retval = tty_register_driver(normal);
if (retval >= 0)
	return retval;


}//newSerialDriver
