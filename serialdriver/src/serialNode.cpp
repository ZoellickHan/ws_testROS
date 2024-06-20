
#include "rclcpp/rclcpp.hpp"

class SerialNode : public rclcpp::Node
{

public:
    
    SerialNode(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "test the serial driver%s.",name.c_str());  
        
    }

private:
   
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialNode>("SerialNode");
    

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
