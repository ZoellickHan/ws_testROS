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

Port::~Port(){}

SerialConfig::~SerialConfig(){}


/**
 * do not use this directly 
*/
bool Port::setBaudRate()
{
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

	return true;
}

/**
 * use this to init the port at first, then open
*/
bool Port::init()
{
    memset(RxBuff,0x00,sizeof(RxBuff));
    memset(TxBuff,0x00,sizeof(TxBuff));

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

// void printHexValues(const std::vector<uint8_t>& vec) {
//     for (auto num : vec) {
//         printf("%02X ", num);
//     }
//     printf("\n");
// }

void Port::registerType(int typeIDArray[ID_NUM],int num)
{
	for(int i=0;i<ID_NUM;i++)
	{
		interestID[i] = typeIDArray[i];
	}
}

/**
 *  Receive the data
*/

// int  Port::decode()
// {	
	
// 	int SIZE;

// 	transform.resize(sizeof(Header));
// 	for(int i = putoutIndex; putoutIndex + sizeof(Header) <= putinIndex; i++ )
// 	{
// 		if(RxBuff[i] != 0xAA) continue;
	
// 		putoutIndex = i;

// 		if((putoutIndex + transform.size()) < putinIndex)
// 			memcpy(transform.data(), RxBuff + putoutIndex, transform.size());
// 		else
// 		{	
// 			memcpy(transform.data(),RxBuff + putinIndex, ROSCOMM_BUFFER_SIZE - putoutIndex);
// 			memcpy(transform.data(),RxBuff,(putoutIndex+transform.size())%ROSCOMM_BUFFER_SIZE);
// 		}
			
// 		header = fromHeaderVector(transform);
// 		crc_ok_header = Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&header), sizeof(header));

		
// 		if(crc_ok_header)
// 		{
// 			for(int j : interestID)
// 			{
// 				switch (j)
// 				{
// 				case GIMBAL_MSG:					
// 					break;
				
// 				case CHASSIS_MSG:
// 					break;
// 				case SENTRY_GIMBAL_MSG:
// 					break;
// 				case FIELD_MSG:
// 					break;


// 				case TWOCRC_GIMBAL_MSG:
// 					Classify(twoCRC_GimbalMsg);
// 					SIZE  = sizeof(twoCRC_GimbalMsg);
// 					break;
// 				case TWOCRC_CHASSIS_MSG:
// 					break;
// 				case TWOCRC_SENTRY_GIMBAL_MSG:
// 					Classify(twoCRC_SentryGimbalMsg);
// 					SIZE = sizeof(twoCRC_SentryGimbalMsg);
// 					break;
// 				case TWOCRC_FIELD_MSG:
// 					break;
// 				default:
// 					break;
// 				}
// 			}
// 		}
// 		else
// 		{
// 			printf("error in header \n");
// 			error_header_count++;
// 		}
			
	
// 		putoutIndex += SIZE;
// 		putoutIndex = putoutIndex>ROSCOMM_BUFFER_SIZE ? putoutIndex%ROSCOMM_BUFFER_SIZE : putoutIndex;

// 		i += SIZE;
// 	}
// 	return decodeCorrectNum;
// }

// template <typename T>
// void Port::Classify(T &data)
// {
// 	T buffer;
// 	transform.resize(sizeof(T));

// 	if((putoutIndex + transform.size()) < putinIndex)
// 		memcpy(transform.data(), RxBuff + putoutIndex, transform.size());
// 	else
// 	{
// 		memcpy(transform.data(),RxBuff + putinIndex, ROSCOMM_BUFFER_SIZE - putoutIndex);
// 		memcpy(transform.data(),RxBuff,(putoutIndex+transform.size())%ROSCOMM_BUFFER_SIZE);
// 	}
// 	buffer = fromTestVector<T>(transform);

// 	crc_ok = Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&buffer), sizeof(T));

// 	if(crc_ok)
// 	{
// 		data = buffer;
// 	}
		
// 	else
// 	{
// 		printf("error in data \n");
// 		error_data_count ++;
// 	}
		
// }

int Port::firstversion_receive()
{
	num_per_read = read(fd,RxBuff,sizeof(RxBuff));

	if(num_per_read > 0)
	{	
		printf("run this\n");
		transform.reserve(150);
		int i=0;
		while (i < num_per_read)
		{
			// printf("run this\n");
			if(RxBuff[i] == 0xAA)
			{	
				transform.resize(sizeof(Header));
				crc_ok_header = crc16::Verify_CRC16_Check_Sum(RxBuff+i,sizeof(Header));
				if(crc_ok_header)
				{
					printf("run that\n");
					memcpy(transform.data(),RxBuff+i,sizeof(header));
					header = fromVector<Header>(transform);
					
					switch (header.protocolID)
					{
					case CommunicationType::CHASSIS_MSG :
						i++;
						break;
					case CommunicationType::FIELD_MSG :
						i++;
						break;
					case CommunicationType::GIMBAL_MSG :
						i++;
						break;
					case CommunicationType::SENTRY_GIMBAL_MSG :
						i++;
						break;
					case CommunicationType::TWOCRC_CHASSIS_MSG :
						i++;
						break;
					case CommunicationType::TWOCRC_FIELD_MSG :
						i++;
						break;
					case CommunicationType::TWOCRC_GIMBAL_MSG :
						printf("id : TWOCRC_GIMBAL_MSG \n");
						transform.resize(sizeof(TwoCRC_GimbalMsg));
						memcpy(transform.data(),RxBuff+i,sizeof(TwoCRC_GimbalMsg));
						if(crc_ok){
							twoCRC_GimbalMsg = fromVector<TwoCRC_GimbalMsg>(transform);
							i+=sizeof(TwoCRC_GimbalMsg);
						}else{
							i+=sizeof(Header);
							error_data_count ++;
						}
						break;
					case CommunicationType::TWOCRC_SENTRY_GIMBAL_MSG :
						printf("id : TWOCRC_SENTRY_GIMBAL_MSG \n");
						memcpy(transform.data(),RxBuff+i,sizeof(TwoCRC_SentryGimbalMsg));
						if(crc_ok){
							twoCRC_SentryGimbalMsg = fromVector<TwoCRC_SentryGimbalMsg>(transform);
							i+=sizeof(TwoCRC_SentryGimbalMsg);
						}else{
							i+=sizeof(Header);
							error_data_count ++;
						}					
						break;
					default:
						i++;
						break;
					}
				}else{
					error_header_count++;
					i += sizeof(Header);
				} // if(crc_ok_header)
				
			}else{
				i++;
				continue;
			}
		} //while
		printf("run out\n");
		return num_per_read;
	}else{
		return -1;
	}
}

// int Port::receive()
// {   
// 	if(ROSCOMM_BUFFER_SIZE - putinIndex < DANGEROUS)
// 	{
// 		num_per_read = read(fd,RxBuff,sizeof(RxBuff));
// 		putinIndex = 0;
// 	}
// 	else
// 	{
// 		num_per_read = read(fd,RxBuff+putinIndex,sizeof(RxBuff));
// 	}	

// 	if(num_per_read > 0)
// 	{	
// 		sum += num_per_read;	
// 		putinIndex += num_per_read;
// 		putinIndex = putinIndex > ROSCOMM_BUFFER_SIZE ? putinIndex%ROSCOMM_BUFFER_SIZE : putinIndex ;
		
// 		printf("haha receive[single:%d ,putinIndex:%d] \n",num_per_read,putinIndex);
//     	return num_per_read;	
// 	}
// 	else
// 	{
// 		return -1;
// 	}
// }
		

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

// bool Port::setFlowControl(bool isFlowControl)
// {
//     struct termios newtio;
// 	memset(&newtio, 0, sizeof(newtio));

//     if (isFlowControl)
// 		newtio.c_cflag |= CRTSCTS;
// 	else
// 		newtio.c_cflag &= ~CRTSCTS;

// 	newtio.c_cc[VTIME] = 10; /* Time-out value (tenths of a second) [!ICANON]. */
// 	newtio.c_cc[VMIN] = 0;	 /* Minimum number of bytes read at once [!ICANON]. */

// 	tcflush(fd, TCIOFLUSH);

// 	if (tcsetattr(fd, TCSANOW, &newtio) != 0) 
// 	{
// 		perror("tcsetattr");
// 		return false;
// 	}
//     return true;
// }

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

}//newSerialDriver