#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "adahil_interface/msg/sensor_data.hpp"

class SensorSend : public rclcpp::Node
{
	public:
		//构造函数
		SensorSend(std::string name) : Node(name)
		{
			RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
			sensor_data_sub = this->create_subscription<adahil_interface::msg::SensorData>("sensor_data", 10, std::bind(&SensorSend::sub_sensor_data_callback, this, std::placeholders::_1) );
		}
	private:
		// 初始化节点
		// 声明订阅者
		rclcpp::Subscription<adahil_interface::msg::SensorData>::SharedPtr sensor_data_sub;
		void sub_sensor_data_callback(const adahil_interface::msg::SensorData::SharedPtr msg)
		{
			printf("sensor data: %x\n", msg->gyrox_h);
		}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<SensorSend>("sensor_send");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world adahil_sensor_send package\n");
//   return 0;
// }
