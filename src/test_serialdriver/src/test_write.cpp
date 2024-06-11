
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

shared_ptr<SerialConfig> config = make_shared<SerialConfig>(2000000,8,0,StopBit::TWO,Parity::NONE);
shared_ptr<Port>         port   = make_shared<Port>(config);
struct timespec ts2;

int single          = 0;
int sum             = 0;
int writeCount      = 0;
double last_time    = 0;
double period       = 0;
double throughput   = 0;
double tiems        = 0;

TwoCRC_GimbalCommand twoCRC_GimbalCommand;
TwoCRC_ChassisCommand twoCRC_ChassisCommand; 

void printBinary(const std::vector<uint8_t>& data) 
{
    for (uint8_t num : data) 
    {
        std::cout << std::bitset<8>(num) << ' ';
    }
    std::cout << std::endl;
}

int main(int argc, char **argv)
{

    twoCRC_ChassisCommand.header.dataLen = sizeof(TwoCRC_ChassisCommand) - sizeof(Header) - 2;
    twoCRC_ChassisCommand.header.protocolID = 0xB2;

    twoCRC_GimbalCommand.header.dataLen = sizeof(TwoCRC_GimbalCommand) - sizeof(Header) - 2;
    twoCRC_GimbalCommand.header.protocolID = 0xB3;


    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("testing");
    
    RCLCPP_INFO(node->get_logger(), "Start ch3se43 in node testing.");
    port->openPort();
    port->init();
    
    clock_gettime(CLOCK_MONOTONIC, &ts2);
    last_time = ts2.tv_sec;

    for(int i =0; 1==1; i++)
    {
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        
        Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&twoCRC_GimbalCommand.header), 5);
        Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&twoCRC_ChassisCommand.header), sizeof(Header));
        twoCRC_GimbalCommand.shoot_mode = i%99;
        twoCRC_GimbalCommand.target_pitch = 0.4;
        twoCRC_GimbalCommand.target_yaw = 0.5;
        Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&twoCRC_GimbalCommand), 16);

        twoCRC_ChassisCommand.vel_w = i%99 ;
        twoCRC_ChassisCommand.vel_x = 0.5;
        twoCRC_ChassisCommand.vel_y = 0.5;  
        Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&twoCRC_ChassisCommand), sizeof(TwoCRC_ChassisCommand));

        std::vector<uint8_t> data1 = toVector(twoCRC_GimbalCommand);
        std::vector<uint8_t> data2 = toVector(twoCRC_ChassisCommand);
        
        single = port->transmit(data1);
        if(single < 0){
            if(port->reopen()) printf("reopen correctly \n");
            else printf("reopen failed \n");
        }
        else{
            sum += single;
            writeCount ++;
        }
       

        // single = port->transmit(data2);
        // if(single < 0){
        //     if(port->reopen()) printf("reopen correctly \n");
        //     else printf("reopen failed \n");
        // }
        // sum += single;
        // writeCount ++;

        clock_gettime(CLOCK_MONOTONIC, &ts2);
        period = ts2.tv_sec - last_time;
        
        if(period != 0) throughput = sum/period;

        printf("time: %f, throuhtput: %f , transmit times: %d, hz:%f \n",period,throughput,writeCount, (float)writeCount/(float)period);
        printf("transmit in total : %d \n", sum);
        printf("data length :%d \n",twoCRC_GimbalCommand.header.dataLen);
        printf("sum: %d,singel :%d, crc1 = %d  crc2 = %d crc3 = %d crc = %d \n",sum,single,twoCRC_GimbalCommand.header.crc_1,twoCRC_GimbalCommand.header.crc_2,twoCRC_GimbalCommand.crc_3,twoCRC_GimbalCommand.crc_4);
        // printBinary(data);
    }   

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}