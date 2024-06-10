
#include "rclcpp/rclcpp.hpp"
#include "newDriver.hpp"
#include <time.h>
#include "protocol.hpp"
#include "crc.hpp"
#include <bitset>
using namespace std;
using namespace newSerialDriver;
using namespace rm_serial_driver;
using namespace crc16;

shared_ptr<SerialConfig> config = make_shared<SerialConfig>(2000000,8,0,StopBit::ONE,Parity::NONE);
shared_ptr<Port>         port   = make_shared<Port>(config);
struct timespec ts1;

int    single       = 0;
int    sum          = 0;
double last_time    = 0;
double period       = 0;
double throughput   = 0;

void printBinary(const std::vector<uint8_t>& data) 
{
    for (uint8_t num : data) 
    {
        std::cout << std::bitset<8>(num) << ' ';
    }
    std::cout << std::endl;
}

void printHexValues(const std::vector<uint8_t>& vec) {
    for (auto num : vec) {
        printf("%02X ", num);
    }
    printf("\n");
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("testing");
    RCLCPP_INFO(node->get_logger(), "Start ch343 in node testing.");

    port->openPort();
    port->init();
    
    rm_serial_driver::TwoCRC_GimbalMsg&  twoCRC_GimbalMsg = port->getTwoCRC_GimbalMsg();
    rm_serial_driver::TwoCRC_SentryGimbalMsg& twoCRC_SentryGimbalMsg = port->getTwoCRC_SentryGimbalMsg();
    int& crcError_header  = port->geterrorHeader();
    int& crcError_data    = port->geterrorData();
    // clock_gettime(CLOCK_MONOTONIC, &ts1);
    // last_time = ts1.tv_sec;

    while(true)
    {

        single = port->firstversion_receive();
        if(single >0)
            sum += single;
        else    
            port->reopen();


        printf("crc1:%d ,crc2:%d ,single: %d sum: %d \n",crcError_header,crcError_data,single,sum);
        printf("normal gimbal:%f \n",twoCRC_GimbalMsg.bullet_speed);
        // printf("crc1: %d, sum: %ld  \n",crcError_header,sum_count);
        printf("two_sentry_gimbal 2: %f \n",twoCRC_SentryGimbalMsg.bullet_speed);
        // printf("two_gimbal 2: %d \n",twoCRC_SentryGimbalMsg.cur_cv_mode);
        // printf("two_gimbal 2: %f \n",twoCRC_SentryGimbalMsg.big_q_w);
        // printf("two_gimbal 2: %f \n",twoCRC_SentryGimbalMsg.big_q_x);
        // printf("two_gimbal 2: %f \n",twoCRC_SentryGimbalMsg.big_q_y);
        // printf("two_gimbal 2: %f \n",twoCRC_SentryGimbalMsg.big_q_z);
    }   


    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}