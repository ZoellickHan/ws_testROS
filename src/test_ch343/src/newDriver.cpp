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

#include <time.h>
#include <chrono>
#include <iostream>
#include <thread>

#define termios asmtermios
#include <asm/termios.h>
#undef termios
#include <termios.h>
namespace rm_serial_driver
{
namespace newSerialDriver
{

using namespace std;
using Parity = SerialConfig::Parity;
using StopBit = SerialConfig::StopBit;
extern "C" int ioctl(int d, int request, ...);

Port::Port(std::shared_ptr<newSerialDriver::SerialConfig> ptr)
{	
	config = ptr;
	memset(TxBuff, 0x00, ROSCOMM_BUFFER_SIZE);
	memset(RxBuff ,0x00, ROSCOMM_BUFFER_SIZE);
}

/**
 * use this to init the port first, then open
*/
bool Port::init()
{
    memset(RxBuff,0x00,sizeof(RxBuff));
    memset(TxBuff,0x00,sizeof(TxBuff));
	transform.reserve(512);

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

/**
 * use a thread to start receive funcation
 * e.g. std::thread receiverThread(&newSerialDriver::Port::receive(),port)
 */
void Port::receive()
{
	while(true)
	{
		num_per_read = read(fd,ReadBuffer,BUFFER_SIZE);
		if(num_per_read>=0)	this->putinIndexHandle(num_per_read);
	}
}

void Port::putinIndexHandle(int size)
{	
	if( putinIndex + size > ROSCOMM_BUFFER_SIZE)
	{
		memcpy(RxBuff+putinIndex,ReadBuffer,putinIndex+size-ROSCOMM_BUFFER_SIZE);
		memcpy(RxBuff,ReadBuffer+putinIndex+size-ROSCOMM_BUFFER_SIZE,(putinIndex+size)%ROSCOMM_BUFFER_SIZE);
		putinIndex = (putinIndex + size)%ROSCOMM_BUFFER_SIZE;
	}
	else
	{
		memcpy(RxBuff+putinIndex,ReadBuffer,size);
		putinIndex += size; 
		putinIndex = putinIndex%ROSCOMM_BUFFER_SIZE;
	}	
}

void Port::jumpHandle(int length1,int length2)
{	
	// printf("boji jump jump jump\n");
	//start the jump buffer !

	memcpy(jumpBuff,RxBuff+putoutIndex +1 ,length1);
	memcpy(jumpBuff+length1,RxBuff,length2);

	for(int i = 0; i< length1+length2; i++)
	{
		if(jumpBuff[i] == 0xAA)
		{	
			if(i + sizeof(Header) >= length1 + length2 ) break;
			crc_ok_header = crc16::Verify_CRC16_Check_Sum(jumpBuff+i,sizeof(Header));
			if(crc_ok_header)
			{
				transform.resize(sizeof(Header));
				memcpy(transform.data(),jumpBuff+i,sizeof(Header));
				header = fromVector<Header>(transform);
				switch (header.protocolID)
				{
				case CommunicationType::TWOCRC_SENTRY_GIMBAL_MSG :
					if(i + sizeof(TwoCRC_SentryGimbalMsg) >= length1+length2) return;
					memcpy(decodeBuffer,jumpBuff+i,sizeof(TwoCRC_SentryGimbalMsg));
					ifdecode = decodeHandle(CommunicationType::TWOCRC_SENTRY_GIMBAL_MSG);
					i += sizeof(TwoCRC_SentryGimbalMsg);
					break;

				case CommunicationType::TWOCRC_GIMBAL_MSG :
					if(i + sizeof(TwoCRC_GimbalMsg) >= length1+length2) return;
					memcpy(decodeBuffer,jumpBuff+i,sizeof(TwoCRC_GimbalMsg));
					ifdecode = decodeHandle(CommunicationType::TWOCRC_GIMBAL_MSG);
					i += sizeof(TwoCRC_GimbalMsg);
					break;
	
				default:
					printf("protocol has not been defined yet!!! \n");
					break;
				}
			}
			else
			{
				i += sizeof(header);
			}
		}
	}
	// printf("boji jump out !\n");
	putoutIndex = length2 -1 ;
	memset(jumpBuff,0x00,sizeof(jumpBuff));
}

void Port::putoutIndexHandle()
{
	while(true)
	{
		if(putoutIndex < putinIndex)
		{
			for(int i = putoutIndex; i < putinIndex; i++)
			{
				if(RxBuff[i] == 0xAA)
				{
					if(i+sizeof(Header)-1 >= putinIndex) continue;
					crc_ok_header = crc16::Verify_CRC16_Check_Sum(RxBuff+i,sizeof(Header));
					if(crc_ok_header)
					{
						transform.resize(sizeof(Header));
						memcpy(transform.data(),RxBuff+i,sizeof(Header));
						header = fromVector<Header>(transform);
						// printf("id: %d \n",header.protocolID);
						switch (header.protocolID)
						{
						case CommunicationType::TWOCRC_SENTRY_GIMBAL_MSG :
							if(i+sizeof(TwoCRC_SentryGimbalMsg)-1 >= putinIndex) continue;
							memcpy(decodeBuffer,RxBuff+i,sizeof(TwoCRC_SentryGimbalMsg));
							ifdecode = decodeHandle(CommunicationType::TWOCRC_SENTRY_GIMBAL_MSG);
							i += sizeof(TwoCRC_SentryGimbalMsg);
							putoutIndex = i - 1;
							putoutIndex = putoutIndex % ROSCOMM_BUFFER_SIZE;
							break;
						
						case CommunicationType::TWOCRC_GIMBAL_MSG :
							if(i+sizeof(TwoCRC_GimbalCommand)-1 >= putinIndex) continue;
							memcpy(decodeBuffer,RxBuff+i,sizeof(TwoCRC_GimbalMsg));
							ifdecode = decodeHandle(CommunicationType::TWOCRC_GIMBAL_MSG);
							i += sizeof(TwoCRC_GimbalMsg);
							putoutIndex = i - 1;
							putoutIndex = putoutIndex % ROSCOMM_BUFFER_SIZE;
							break;

 						default:
							printf("protocol has not been defined yet.\n");
							continue;
							break;
						}
					}
					else
					{
						error_header_count ++;
						i += sizeof(header);
						putoutIndex = i -1 ;
						putoutIndex = putoutIndex % ROSCOMM_BUFFER_SIZE;
					}
				}
				else
				{
					i++;
				}
			}
		}
		else
		{
			jumpHandle(ROSCOMM_BUFFER_SIZE - putoutIndex - 1, putinIndex);
		}
	}
}


bool Port::decodeHandle(int ID)
{
	switch (ID)
	{
	case CommunicationType::TWOCRC_SENTRY_GIMBAL_MSG :

		crc_ok = crc16::Verify_CRC16_Check_Sum(decodeBuffer,sizeof(TwoCRC_SentryGimbalMsg));
		if(crc_ok)
		{
			transform.resize(sizeof(TwoCRC_SentryGimbalMsg));
			memcpy(transform.data(),decodeBuffer,sizeof(TwoCRC_SentryGimbalMsg));
			twoCRC_SentryGimbalMsg = fromVector<TwoCRC_SentryGimbalMsg>(transform);
			decodeCount ++;
			return true;
		}
		else
		{
			error_data_count++;
			return false;
		}
		break;

	case CommunicationType::TWOCRC_GIMBAL_MSG :
		crc_ok = crc16::Verify_CRC16_Check_Sum(decodeBuffer,sizeof(TwoCRC_GimbalMsg));
		if(crc_ok)
		{
			transform.resize(sizeof(TwoCRC_GimbalMsg));
			memcpy(transform.data(),decodeBuffer,sizeof(TwoCRC_GimbalMsg));
			twoCRC_GimbalMsg = fromVector<TwoCRC_GimbalMsg>(transform);
			decodeCount ++;
			return true;
		}
		else
		{
			error_data_count++;
			return false;
		}
		break;
	
	default:
		return false;
		break;
	}
	memset(decodeBuffer,0x00,sizeof(decodeBuffer));
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

Port::~Port()
{
	if(receiveThread.joinable())
		receiveThread.join();
	if(decodeThread.joinable())
		decodeThread.join();
	if(isopen)
		closePort();
}
SerialConfig::~SerialConfig(){}

}//newSerialDriver
}//rm_serial_driver