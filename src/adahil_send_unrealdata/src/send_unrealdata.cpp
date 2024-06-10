#include <cstdio>
#include "adahil_interface/msg/unreal_display_data.hpp"
#include "rclcpp/rclcpp.hpp"

class UnrealDataSend : public rclcpp::Node
{
public:
	UnrealDataSend(std::string name) : Node(name)
	{
		RCLCPP_INFO(this->get_logger(), "[unreal data send] node is running: %s.", name.c_str());
	}
private:

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<UnrealDataSend>("unreal_data_send");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
