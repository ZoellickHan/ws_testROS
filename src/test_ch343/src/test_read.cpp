
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
    Header headerrrrrrr;
    TwoCRC_SentryGimbalMsg twoCRC_SentryGimbalMsg;
    vector<uint8_t> header(5);
    vector<uint8_t> data;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("testing");
    RCLCPP_INFO(node->get_logger(), "Start ch343 in node testing.");

    port->openPort();
    port->init();
    
    clock_gettime(CLOCK_MONOTONIC, &ts1);
    last_time = ts1.tv_sec;
    //AA 16 20 DD A5 ->  221 165
    while(true)
    {
        data.reserve(sizeof(TwoCRC_SentryGimbalMsg));
        single = port -> receive(header);
        // if(single != 5  ) printf("not happy boji =( for that %d \n",single);
        headerrrrrrr = fromHeaderVector(header);
        bool crc_ok_header = Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>
        (&headerrrrrrr), sizeof(headerrrrrrr));
        printf("singel : %d \n",single);
        // printHexValues(header);
        // printf("header:%d,%d,%d,%d,%d \n",headerrrrrrr.sof,headerrrrrrr.dataLen,headerrrrrrr.protocolID,headerrrrrrr.crc_1,headerrrrrrr.crc_2);

        if(!crc_ok_header)
        {
            printf("crc error in header! \n");
            continue;
        }
   
            // sum += single;
            data.resize(sizeof(TwoCRC_SentryGimbalMsg));
            single = port->receive(data);
            // printHexValues(data);
            twoCRC_SentryGimbalMsg = fromSentryTwoCRCVector(data);
            
            // printf("crc3: %d \n", twoCRC_SentryGimbalMsg.crc_3);
            // printf("crc4: %d \n", twoCRC_SentryGimbalMsg.crc_4);

            // printf("data:%d,%f,%d \n",twoCRC_SentryGimbalMsg.target_color,twoCRC_SentryGimbalMsg.bullet_speed,twoCRC_SentryGimbalMsg.cur_cv_mode);

            bool crc_ok = Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>
            (&twoCRC_SentryGimbalMsg), sizeof(twoCRC_SentryGimbalMsg));
            
            // if(crc_ok&&crc_ok_header) printf("BOJI!!! %d \n",single);
            if(crc_ok)
                sum += single;
            else
            {
                printf("crc error ! \n");
                continue;
            }
        
        
            
        

        
        // printf("TwoCRC_SentryGimbalMsg:%d \n",twoCRC_SentryGimbalMsg.dataLen);
        // if (!crc_ok) 
        // {
        //     errorCounter += single;
        //     crcError++;
        //     printf("crc_not ok\n");
        //     continue;
        // }
    
        
        // clock_gettime(CLOCK_MONOTONIC, &ts1);
        // period = ts1.tv_sec - last_time;
        
        // if(sum != 0) errorRate = (double)errorCounter / (double)sum;
        // if(period != 0){ throughput = 10*sum/period; } 

        // printf("time: %f, throuhtput: %f, errorRate: %f, crcError:%d \n",period,throughput,errorRate,crcError);

    }   

    printf("time: %f, throuhtput: %f, errorRate: %f \n",period,throughput,errorRate);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}