
#include "rclcpp/rclcpp.hpp"
#include "newDriver.hpp"
#include <time.h>
#include <bitset>
#include <chrono>
#include <thread>
using namespace std;
using namespace newSerialDriver;
using namespace rm_serial_driver;
using namespace crc16;
using namespace chrono;

shared_ptr<SerialConfig> config = make_shared<SerialConfig>(2000000,8,0,StopBit::TWO,Parity::NONE);
shared_ptr<Port>         port   = make_shared<Port>(config);


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("testing");
    RCLCPP_INFO(node->get_logger(), "Start ch343 in node testing.");

    port->openPort();
    port->init();
    
    rm_serial_driver::TwoCRC_GimbalMsg&  twoCRC_GimbalMsg = port->getTwoCRC_GimbalMsg();
    rm_serial_driver::TwoCRC_SentryGimbalMsg& twoCRC_SentryGimbalMsg = port->getTwoCRC_SentryGimbalMsg();
    int&    crcError_header  = port->geterrorHeader();
    int&    crcError_data    = port->geterrorData();
    int&    decodeCount      = port->getdecodeCount();
    long&   sum           = port->getsum();
    int&    putinIndex    = port->getputinIndex();
    int&    putoutIndex   = port->getputoutIndex();
    auto    start            = high_resolution_clock::now();

    std::thread receiveThread(&newSerialDriver::Port::receive,port);
    std::thread putoutHandle(&newSerialDriver::Port::putoutIndexHandle,port);

    while(true)
    {
        auto stop =  high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);

        printf("[sum: %ld, crc1: %d, crc2: %d, putin: %d, putout: %d, decode: %d] \n"
        ,sum,crcError_header,crcError_data,putinIndex,putoutIndex,decodeCount);
        // printf("single :%d, sum: %d, decodeRate: %f, time: %f \n",single,sum,float(decodenum)/double(duration.count())*1000000.0,double(duration.count())/1000000.0);
        

        // printf("two_sentry_gimbal 2: %d \n",twoCRC_SentryGimbalMsg.header.dataLen);
        // printf("two_sentry_gimbal 2: %d \n",twoCRC_SentryGimbalMsg.header.protocolID);
        // printf("two_sentry_gimbal 2: %d \n",twoCRC_SentryGimbalMsg.header.crc_1);
        // printf("two_sentry_gimbal 2: %d \n",twoCRC_SentryGimbalMsg.header.crc_2);
        // printf("two_sentry_gimbal 2: %d \n",twoCRC_SentryGimbalMsg.cur_cv_mode);
        // printf("two_sentry_gimbal 2: %f \n",twoCRC_SentryGimbalMsg.bullet_speed);
        // printf("two_sentry_gimbal 2: %f \n",twoCRC_SentryGimbalMsg.small_q_w);
        // printf("two_sentry_gimbal 2: %f \n",twoCRC_SentryGimbalMsg.small_q_x);
        // printf("two_sentry_gimbal 2: %f \n",twoCRC_SentryGimbalMsg.small_q_y);
        // printf("two_sentry_gimbal 2: %f \n",twoCRC_SentryGimbalMsg.small_q_z);
        // printf("two_sentry_gimbal 2: %f \n",twoCRC_SentryGimbalMsg.big_q_w);
        // printf("two_sentry_gimbal 2: %f \n",twoCRC_SentryGimbalMsg.big_q_x);
        // printf("two_sentry_gimbal 2: %f \n",twoCRC_SentryGimbalMsg.big_q_y);
        // printf("two_sentry_gimbal 2: %f \n",twoCRC_SentryGimbalMsg.big_q_z);
        // printf("two_sentry_gimbal 2: %d \n",twoCRC_SentryGimbalMsg.crc_3);
        // printf("two_sentry_gimbal 2: %d \n",twoCRC_SentryGimbalMsg.crc_4);
        // printf("crc1:%d ,crc2:%d ,single: %d sum: %d,decode_rate :%f ,rate : %f, time: %f \n",crcError_header,crcError_data,single,sum,float(decodeCount)/double(duration.count())*1000000,float(crcError_data+crcError_header)/float(decodeCount),double(duration.count())/1000000);

    }   

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}