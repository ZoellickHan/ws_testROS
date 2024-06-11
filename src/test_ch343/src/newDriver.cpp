#include "newDriver.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <getopt.h>
#include <linux/serial.h>

#define termios asmtermios
#include <asm/termios.h>
#undef termios
#include <termios.h>

namespace newSerialDriver
{
using namespace crc16;
using namespace std;
using namespace newSerialDriver;
using namespace rm_serial_driver;
extern "C" int ioctl(int d, int request, ...);

Port::Port(std::shared_ptr<newSerialDriver::SerialConfig> ptr)
{	
	config = ptr;
	memset(TxBuff, 0x00, ROSCOMM_BUFFER_SIZE);
	memset(RxBuff ,0x00, ROSCOMM_BUFFER_SIZE);
}

/**
 * use this to init the port at first, then open
*/
bool Port::init()
{
    memset(RxBuff,0x00,sizeof(RxBuff));
    memset(TxBuff,0x00,sizeof(TxBuff));
	transform.reserve(DANGEROUS);

    // init port 
    struct termios newtio;
	struct termios oldtio;
    bzero(&newtio, sizeof(newtio));
	bzero(&oldtio, sizeof(oldtio));

    if (tcgetattr(fd, &oldtio) != 0) 
    {
		perror("tcgetattr");
		return false;
	}
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

    	/* set data bits */
	switch (config->databits) 
	{
	case 5:
		newtio.c_cflag |= CS5;
		break;
	case 6:
		newtio.c_cflag |= CS6;
		break;
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	default:
		fprintf(stderr, "unsupported data size\n");
		return false;
	}
    /* set parity */
	switch (config->parity) 
    {
	case Parity::NONE :
		newtio.c_cflag &= ~PARENB; /* Clear parity enable */
		newtio.c_iflag &= ~INPCK;  /* Disable input parity check */
		break;
	case Parity::ODD :
		newtio.c_cflag |= (PARODD | PARENB); /* Odd parity instead of even */
		newtio.c_iflag |= INPCK;	     /* Enable input parity check */
		break;
    case Parity::EVEN :
		newtio.c_cflag |= PARENB;  /* Enable parity */
		newtio.c_cflag &= ~PARODD; /* Even parity instead of odd */
		newtio.c_iflag |= INPCK;   /* Enable input parity check */
		break;
	case Parity::MARK :
		newtio.c_cflag |= PARENB; /* Enable parity */
		newtio.c_cflag |= CMSPAR; /* Stick parity instead */
		newtio.c_cflag |= PARODD; /* Even parity instead of odd */
		newtio.c_iflag |= INPCK;  /* Enable input parity check */
		break;
    case Parity::SPACE :
		newtio.c_cflag |= PARENB;  /* Enable parity */
		newtio.c_cflag |= CMSPAR;  /* Stick parity instead */
		newtio.c_cflag &= ~PARODD; /* Even parity instead of odd */
		newtio.c_iflag |= INPCK;   /* Enable input parity check */
		break;
	default:
		fprintf(stderr, "unsupported parity\n");
		return false;
	}

	/* set stop bits */
	switch (config->stopbit) 
	{
	case StopBit::ONE :
		newtio.c_cflag &= ~CSTOPB;
		break;
	case StopBit::TWO :
		newtio.c_cflag |= CSTOPB;
		break;
	default:
		perror("unsupported stop bits\n");
		return false;
	}

	if (config->flowcontrol)
		newtio.c_cflag |= CRTSCTS;
	else
		newtio.c_cflag &= ~CRTSCTS;

	newtio.c_cc[VTIME] = 10; /* Time-out value (tenths of a second) [!ICANON]. */
	newtio.c_cc[VMIN] = 0;	 /* Minimum number of bytes read at once [!ICANON]. */
	tcflush(fd, TCIOFLUSH);

	if (tcsetattr(fd, TCSANOW, &newtio) != 0) 
	{
		perror("tcsetattr");
		return false;
	}

	struct termios2 tio;

	if (ioctl(fd, TCGETS2, &tio)) 
	{
		perror("TCGETS2");
		return false;
	}

	tio.c_cflag &= ~CBAUD;
	tio.c_cflag |= BOTHER;
	tio.c_ispeed = config->baudrate;
	tio.c_ospeed = config->baudrate;

	if (ioctl(fd, TCSETS2, &tio)) 
	{
		perror("TCSETS2");
		return false;
	}

	if (ioctl(fd, TCGETS2, &tio)) 
	{
		perror("TCGETS2");
		return false;
	} 
	isinit = true;
    return true;
}

int Port::openPort()
{	
    int flags = 0;
    fd = open(config->devname, O_RDWR | O_NOCTTY | O_NDELAY); 

	if (fd < 0) 
	{                        
		perror("open device failed");
		isopen = false;          
	}
	
	flags = fcntl(fd, F_GETFL, 0);
	flags &= ~O_NONBLOCK;

	if (fcntl(fd, F_SETFL, flags) < 0) 
	{
		printf("fcntl failed.\n");
		isopen = false;
	}
		
	if (isatty(fd) == 0) 
	{
		printf("not tty device.\n");
		isopen = false;
	}

	else 
	{
		printf("tty device test ok.\n");
        isopen = true;
		init();
	}
	
	return fd;
}

int Port::firstversion_receive()
{
	//reset the buffer
	memcpy(RxBuff,RxBuff+putoutIndex,putinIndex-putoutIndex);
	putinIndex = putinIndex - putoutIndex;
	putoutIndex = 0;
	memset(RxBuff+putinIndex,0x00,DANGEROUS);

	printf("putin index1: %d \n",putinIndex);
	printf("putout index1: %d \n",putoutIndex);

	num_per_read = read(fd,RxBuff+putinIndex,1024);
	if(num_per_read != 64) printf("!!!!!!!!!!! num_per_read :%d \n",num_per_read);
	if(num_per_read >= 0)
	{	
		putinIndex += num_per_read;
		// if receiver lass than a header. then decode next time.
		if(putinIndex - putoutIndex < sizeof(Header)){return num_per_read;}

		int i=0;
		while (i < num_per_read )
		{
			if(RxBuff[i+putoutIndex] == 0xAA)
			{	
				putoutIndex += i; // update the putout when searching.
				transform.resize(sizeof(Header));
				crc_ok_header = crc16::Verify_CRC16_Check_Sum(RxBuff+i,sizeof(Header));
				if(crc_ok_header)
				{
					memcpy(transform.data(),RxBuff+i,sizeof(header));
					header = fromVector<Header>(transform);
					switch (header.protocolID)
					{
					case CommunicationType::TWOCRC_CHASSIS_MSG :
						/*not define*/
						i++;
						break;
					case CommunicationType::TWOCRC_FIELD_MSG :
						/*not define*/
						i++;
						break;
					case CommunicationType::TWOCRC_GIMBAL_MSG :
						if(putinIndex - putoutIndex < sizeof(TwoCRC_GimbalMsg)){break;}
						crc_ok = crc16::Verify_CRC16_Check_Sum(RxBuff+i,sizeof(TwoCRC_GimbalMsg));
						if(crc_ok)
						{
							transform.resize(sizeof(TwoCRC_GimbalMsg));
							memcpy(transform.data(),RxBuff+i,sizeof(TwoCRC_GimbalMsg));
							twoCRC_GimbalMsg = fromVector<TwoCRC_GimbalMsg>(transform);
							decodeCount++;
						}
						else
						{
							error_data_count ++;
						}
						i+=sizeof(TwoCRC_GimbalMsg);
						putoutIndex += sizeof(TwoCRC_GimbalMsg);
						break;

					case CommunicationType::TWOCRC_SENTRY_GIMBAL_MSG :
						if(putinIndex - putoutIndex < sizeof(TwoCRC_SentryGimbalMsg)){break;}
						crc_ok = crc16::Verify_CRC16_Check_Sum(RxBuff+i,sizeof(TwoCRC_SentryGimbalMsg));
						if(crc_ok)
						{
							transform.resize(sizeof(TwoCRC_SentryGimbalMsg));
							memcpy(transform.data(),RxBuff+i,sizeof(TwoCRC_SentryGimbalMsg));
							twoCRC_SentryGimbalMsg = fromVector<TwoCRC_SentryGimbalMsg>(transform);
							decodeCount++;
						}
						else
						{
							error_data_count ++;
						}
						putoutIndex += sizeof(TwoCRC_SentryGimbalMsg);	
						i+=sizeof(TwoCRC_SentryGimbalMsg);				
						break;
					default:
						i++;
						break;
					}
				}
				else
				{
					error_header_count++;
					i += sizeof(Header);
				} // if(crc_ok_header)
			}
			else
			{
				i++;
				continue;
			}
		} //while
		printf("putin index2: %d \n",putinIndex);
		printf("putout index2: %d \n",putoutIndex);
 		printf("run out\n");
		return num_per_read;
	}else{
		return -1;
	}
}

/**
 *  Transmit the data
*/
int Port::transmit(std::vector<uint8_t> & buff)
{
	memcpy(&TxBuff, buff.data() ,buff.size());
    int num_per_write = write(fd,TxBuff,buff.size());

	if(num_per_write > 0)
		return num_per_write;
	else
		return -1;	
}

/**
 *  Close the port
*/
bool Port::closePort()
{
    return close(fd);
}

bool Port::reopen()
{
	if(isPortOpen()) closePort();

	if(openPort()) return true;
	else
	{
		config->devname = (char*)"/dev/ttyCH343USB0";
		if(openPort()){	return true; }
		else{
			config->devname = (char*)"/dev/ttyCH343USB1";
			if(openPort()) {return true;}
			else{
				config->devname = (char*)"/dev/ttyCH343USB2";
				if(openPort()) return true;
				else return false;
			}
		}

	}
}

bool Port::isPortInit()
{
	if(isinit)
		return true;
	else
		return false;
}

bool Port::isPortOpen()
{
	if(isopen)
		return true;
	else
		return false;
}

Port::~Port(){}
SerialConfig::~SerialConfig(){}

}//newSerialDriver