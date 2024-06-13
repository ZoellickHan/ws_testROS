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

using namespace std;
using namespace newSerialDriver;
using namespace rm_serial_driver;
using StopBit = newSerialDriver::SerialConfig::StopBit;
using Parity  = newSerialDriver::SerialConfig::Parity;
extern "C" int ioctl(int d, int request, ...);

Port::Port(std::shared_ptr<newSerialDriver::SerialConfig> ptr)
{	
	config = ptr;
	memset(TxBuff, 0x00, ROSCOMM_BUFFER_SIZE);
	memset(RxBuff, 0x00, ROSCOMM_BUFFER_SIZE);
	memset(readBuffer, 0x00, BUFFER_SIZE);
}

/**
 * use this to init the port at first, then open
*/
bool Port::init()
{
    memset(RxBuff,0x00,ROSCOMM_BUFFER_SIZE);
    memset(TxBuff,0x00,ROSCOMM_BUFFER_SIZE);
	memset(readBuffer,0x00,BUFFER_SIZE);
	transformVector.reserve(BUFFER_SIZE);

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

void Port::readFun()
{	
	while(true)
	{
		num_per_read = read(fd,readBuffer,BUFFER_SIZE);
		putinIndexFun(num_per_read);
	}
}

void Port::putinIndexFun(int size)
{
	if( putinIndex + size > ROSCOMM_BUFFER_SIZE)
	{
		memcpy(RxBuff+putinIndex, readBuffer, ROSCOMM_BUFFER_SIZE-putinIndex);
		memcpy(RxBuff, readBuffer+(ROSCOMM_BUFFER_SIZE-putinIndex), size - (ROSCOMM_BUFFER_SIZE-putinIndex));
		putinIndex = (putinIndex + size) % ROSCOMM_BUFFER_SIZE;
	}
	else
	{
		memcpy(RxBuff+putinIndex,readBuffer,size);
		putinIndex += size;
	}
	rxsize = (putinIndex - putoutIndex + ROSCOMM_BUFFER_SIZE) % ROSCOMM_BUFFER_SIZE;
}

Port::PkgState Port::putoutIndexFun()
{
	int size = rxsize;
	int putout = putoutIndex;

	while(size > 2)
	{		
		if(RxBuff[putout] == 0xAA)
		{
			break;
		}
		putout = (putout + 1)%ROSCOMM_BUFFER_SIZE;
		size-- ;
	}

	// check the header size
	if( size < sizeof(Header))
	{
		frameState = PkgState::HEADER_INCOMPLETE;
		return PkgState::HEADER_INCOMPLETE;
	}

	// copy the header
	if( putout + sizeof(Header) < ROSCOMM_BUFFER_SIZE )
	{
		memcpy(frameBuffer,RxBuff+putout,sizeof(Header));
	}
	else
	{
		memcpy(frameBuffer,RxBuff + putout, ROSCOMM_BUFFER_SIZE - putout);
		memcpy(frameBuffer + (ROSCOMM_BUFFER_SIZE - putout),RxBuff,sizeof(Header)-(ROSCOMM_BUFFER_SIZE - putout));
	}

	putout += sizeof(Header); //update
	crc_ok_header = crc16::Verify_CRC16_Check_Sum(frameBuffer,sizeof(Header));

	if(!crc_ok_header)
	{
		frameState = PkgState::CRC_HEADER_ERRROR;
		putout = (putout + sizeof(Header) -1 ) % ROSCOMM_BUFFER_SIZE;
		size -= sizeof(Header);
		putoutIndex = putout;
	}

	transformVector.resize(sizeof(Header));
	memcpy(transformVector.data(),frameBuffer,sizeof(Header));
	header = fromVector<Header>(transformVector);

	// check the pay load 
	switch (header.protocolID)
	{
		case CommunicationType::TWOCRC_SENTRY_GIMBAL_MSG :

			if(size < sizeof(TwoCRC_SentryGimbalMsg))
			{
				frameState = PkgState::PAYLOAD_INCOMPLETE;
				return PkgState::PAYLOAD_INCOMPLETE;
			}

			//copy the payload
			if( putout + sizeof(TwoCRC_SentryGimbalMsg) - sizeof(Header) < ROSCOMM_BUFFER_SIZE )
			{
				memcpy(frameBuffer+sizeof(Header),RxBuff + putout, sizeof(TwoCRC_SentryGimbalMsg)-sizeof(Header));
			} 
			else
			{
				memcpy(frameBuffer+sizeof(Header),RxBuff+putout, ROSCOMM_BUFFER_SIZE - putout);
				memcpy(frameBuffer+(sizeof(Header)+ROSCOMM_BUFFER_SIZE-putout),RxBuff,sizeof(TwoCRC_SentryGimbalMsg)-sizeof(Header)-(ROSCOMM_BUFFER_SIZE - putout));
			}

			crc_ok = crc16::Verify_CRC16_Check_Sum(frameBuffer,sizeof(TwoCRC_SentryGimbalMsg));

			if(!crc_ok)
			{
				frameState = PkgState::CRC_PKG_ERROR;
				memset(frameBuffer,0x00,BUFFER_SIZE);
			}

			// complete pkg !
			frameState = PkgState::COMPLETE;
			decodeFun();
			putout = (putout+sizeof(TwoCRC_SentryGimbalMsg)-sizeof(Header))%ROSCOMM_BUFFER_SIZE;
			putoutIndex = putout;
			size -= sizeof(TwoCRC_SentryGimbalMsg);
			break;
	
		case CommunicationType::TWOCRC_GIMBAL_MSG :

			if(size < sizeof(TwoCRC_GimbalMsg))
			{
				frameState = PkgState::PAYLOAD_INCOMPLETE;
				return PkgState::PAYLOAD_INCOMPLETE;
			}

			//copy the payload
			if( putout + sizeof(TwoCRC_GimbalMsg) - sizeof(Header) < ROSCOMM_BUFFER_SIZE )
			{
				memcpy(frameBuffer+sizeof(Header),RxBuff+putout, sizeof(TwoCRC_GimbalMsg)-sizeof(Header));
			} 
			else
			{
				memcpy(frameBuffer+sizeof(Header),RxBuff+putout, ROSCOMM_BUFFER_SIZE-putout);
				memcpy(frameBuffer+(sizeof(Header)+ROSCOMM_BUFFER_SIZE-putout),RxBuff,sizeof(TwoCRC_GimbalMsg)-sizeof(Header)-(ROSCOMM_BUFFER_SIZE - putout));
			}

			crc_ok = crc16::Verify_CRC16_Check_Sum(frameBuffer,sizeof(TwoCRC_GimbalMsg));

			if(!crc_ok)
			{
				frameState = PkgState::CRC_PKG_ERROR;
				memset(frameBuffer,0x00,BUFFER_SIZE);
			}

			// complete pkg !
			frameState = PkgState::COMPLETE;
			decodeFun();
			putout = (putout+sizeof(TwoCRC_GimbalMsg)-sizeof(Header))%ROSCOMM_BUFFER_SIZE;
			putoutIndex = putout;
			size -= sizeof(TwoCRC_GimbalMsg);
			break;

		default:
			printf("Protocol is not defined \n");
			break;
	}

}

void Port::decodeFun()
{
	//check the id
	if(frameState != PkgState::COMPLETE)	return;

	switch (frameBuffer[2])
	{
	case CommunicationType::TWOCRC_SENTRY_GIMBAL_MSG :

		transformVector.resize(sizeof(TwoCRC_SentryGimbalMsg));
		twoCRC_SentryGimbalMsg = fromVector<TwoCRC_SentryGimbalMsg>(transformVector);
		break;

	case CommunicationType::TWOCRC_GIMBAL_MSG :
		transformVector.resize(sizeof(TwoCRC_GimbalMsg));
		twoCRC_GimbalMsg = fromVector<TwoCRC_GimbalMsg>(transformVector);
		break;

	default:
		printf("Protocol is not defined \n");
		break;

	memset(frameBuffer,0x00,BUFFER_SIZE);
	}
} 
}//newSerialDriver