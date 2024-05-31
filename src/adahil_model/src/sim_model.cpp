// #include <cstdio>
#include "adahil_interface/msg/sensor_data.hpp"
#include "rclcpp/rclcpp.hpp"

extern "C"{
	#include "libsimodel.h"
}

class SimModel : public rclcpp::Node {
public:
	// 构造函数,有一个参数为节点名称
	SimModel(std::string name) : Node(name) {
		RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
		sensor_data_publisher = this->create_publisher<adahil_interface::msg::SensorData>("sensor_data",5);
		// 创建定时器，xxxms为周期。milliseconds表示毫秒  microseconds表示微妙
		timer_ = this->create_wall_timer(std::chrono::microseconds(1000), std::bind(&SimModel::timer_callback, this));
		libsimodel_initialize();
	}
	
private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<adahil_interface::msg::SensorData>::SharedPtr sensor_data_publisher;
	void timer_callback(){
		//set input
		libsimodel_U.inPWMs[0] = 0;
		libsimodel_U.inPWMs[1] = 0;
		libsimodel_U.inPWMs[2] = 0;
		libsimodel_U.inPWMs[3] = 0;
		//simulation step
		libsimodel_step();
		//set output
		// libsimodel_Y.AccelSensorData
		//创建消息
		adahil_interface::msg::SensorData msg;
		msg.accelx_h = 0x01;
		msg.accelx_l = 0x02;
		msg.accely_h = 0x03;
		msg.accely_l = 0x04;
		msg.accelz_h = 0x05;
		msg.accelz_l = 0x06;
		msg.temp_h = 0x07;
		msg.temp_l = 0x08;
		msg.gyrox_h = 0x09;
		msg.gyrox_l = 0x0A;
		msg.gyroy_h = 0x0B;
		msg.gyroy_l = 0x0C;
		msg.gyroz_h = 0x0D;
		msg.gyroz_l = 0x0E;
		sensor_data_publisher->publish(msg);
		// RCLCPP_INFO(this->get_logger(), "Publishing");
	}
};


int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SimModel>("sim_model");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world adahil_model package\n");
//   return 0;
// }
