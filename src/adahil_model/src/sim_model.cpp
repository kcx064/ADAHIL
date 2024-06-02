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
		// 创建定时器，xxxms为周期。milliseconds 表示毫秒  microseconds 表示微妙
		timer_ = this->create_wall_timer(std::chrono::microseconds(500), std::bind(&SimModel::timer_callback, this));
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
		//创建消息
		adahil_interface::msg::SensorData msg;
		msg.header.frame_id = "sim_model";
		msg.header.stamp = this->get_clock()->now();// builtin_interfaces::msg::Time now = this->get_clock()->now();
		
		split16bitTo8bit(libsimodel_Y.GyroSensorData[0], &msg.gyrox_h, &msg.gyrox_l);
		split16bitTo8bit(libsimodel_Y.GyroSensorData[1], &msg.gyroy_h, &msg.gyroy_l);
		split16bitTo8bit(libsimodel_Y.GyroSensorData[2], &msg.gyroz_h, &msg.gyroz_l);

		split16bitTo8bit(libsimodel_Y.AccelSensorData[0], &msg.accelx_h, &msg.accelx_l);
		split16bitTo8bit(libsimodel_Y.AccelSensorData[1], &msg.accely_h, &msg.accely_l);
		split16bitTo8bit(libsimodel_Y.AccelSensorData[2], &msg.accelz_h, &msg.accelz_l);

		split16bitTo8bit(libsimodel_Y.TempData, &msg.temp_h, &msg.temp_l);

		sensor_data_publisher->publish(msg);
		// RCLCPP_INFO(this->get_logger(), "Publishing");
	}

	void split16bitTo8bit(int16_t input, uint8_t *high, uint8_t *low)
	{
		*high = (input >> 8) & 0xFF;
		*low = input & 0xFF;
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
