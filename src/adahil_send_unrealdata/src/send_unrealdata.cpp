#include <cstdio>
#include <iostream>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "adahil_interface/msg/unreal_display_data.hpp"

class UnrealDataSend : public rclcpp::Node
{
public:
	UnrealDataSend(std::string name) : Node(name)
	{
		RCLCPP_INFO(this->get_logger(), "[unreal data send] node is running: %s.", name.c_str());

		/* 创建回调组, 类型为MutuallyExclusive（默认），即每次只执行一个回调函数。另有一种Reentrant为多线程调用，组内回调函数可能会同时运行。 */
		callback_group_sub1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		auto sub1_opt = rclcpp::SubscriptionOptions();
    	sub1_opt.callback_group = callback_group_sub1_;
		/* 创建订阅者；定义回调函数 */
		unrealdisplay_data_sub = this->create_subscription<adahil_interface::msg::UnrealDisplayData>("unreal_display_data", 10, std::bind(&UnrealDataSend::sub_unrealdisplay_data_callback, this,std::placeholders::_1), sub1_opt);

		/* 创建定时器；定义回调函数 */
		timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&UnrealDataSend::timer_callback, this), callback_group_sub1_);
	}
private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<adahil_interface::msg::UnrealDisplayData>::SharedPtr unrealdisplay_data_sub;
	rclcpp::CallbackGroup::SharedPtr callback_group_sub1_;
	
	/* 定义udp数据包 */
	std::array<uint8_t, 200UL> unrealudp_data;
	
	void timer_callback()
	{
		// RCLCPP_INFO(this->get_logger(), "[unreal data send] timer is running.");
		/* 创建udp发送unrealudp_data到目标ip */
		const char *target_ip = "192.168.0.105"; // 替换为你的目标IP
    	const int target_port = 20010; // 替换为你的目标端口

		int sockfd;
    	struct sockaddr_in serv_addr;

		// 创建UDP套接字
		if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) 
		{
			std::cerr << "Error opening socket" << std::endl;
			exit(1);
		}

		memset((char *) &serv_addr, 0, sizeof(serv_addr));
		serv_addr.sin_family = AF_INET;
		serv_addr.sin_addr.s_addr = inet_addr(target_ip);
		serv_addr.sin_port = htons(target_port);
		// 发送数据
		if (sendto(sockfd, reinterpret_cast<const void*>(unrealudp_data.data()), 200, 0, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
			RCLCPP_ERROR(this->get_logger(), "[unreal data send] udp sendto error.");
			exit(1);
		}
		close(sockfd);
	}

	void sub_unrealdisplay_data_callback(const adahil_interface::msg::UnrealDisplayData::SharedPtr msg)
	{
		// RCLCPP_INFO(this->get_logger(), "[unreal data send] receive unrealdisplay_data");
		std::copy_n(msg->unrealdata.data(), 200, unrealudp_data.begin());
	}

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<UnrealDataSend>("unreal_data_send");
	
	executor.add_node(node);
	executor.spin();
	// rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
