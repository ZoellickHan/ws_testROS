
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

int    okbag;
int    crcError     = 0;
int    errorCounter = 0;
double errorRate    = 0;
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
    
    clock_gettime(CLOCK_MONOTONIC, &ts1);
    last_time = ts1.tv_sec;

    while(true)
    {
        single = port->receive();
        
    }   


    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}