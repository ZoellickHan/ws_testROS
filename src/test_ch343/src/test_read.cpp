
#include "rclcpp/rclcpp.hpp"
#include "newDriver.hpp"
#include <time.h>

int main(int argc, char **argv)
{
    /* 初始化rclcpp  */
    using namespace std;
    using namespace newSerialDriver;

    shared_ptr<SerialConfig> config = make_shared<SerialConfig>(3000000,8,0,StopBit::ONE,Parity::NONE);
    shared_ptr<Port>         port   = make_shared<Port>(config);

    time_t timeptr;
    long last_time=0,period=0;
    double throughput=0,
    time(timeptr);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("testing");
    
    RCLCPP_INFO(node->get_logger(), "Start ch343 in node testing.");
    port->openPort();
    port->init();
    last_time = timeptr;
    while(true)
    {
        port->test_receive();
        period = timeptr - last_time;
        if(period != 0) throughput = port->getNumRead()/period*10000;
    }   

    printf("time: %ld, throuhtput: %f \n",period,throughput);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}