// #include <cstdio>
#include "adahil_interface/msg/sensor_data.hpp"
#include "adahil_interface/msg/mavlink_sensor_data.hpp"
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
		mavlink_sensor_data_publisher = this->create_publisher<adahil_interface::msg::MavlinkSensorData>("mavlink_sensor_data",5);
		// 创建定时器，xxxms为周期。milliseconds 表示毫秒  microseconds 表示微妙
		timer_ = this->create_wall_timer(std::chrono::microseconds(500), std::bind(&SimModel::timer_callback, this));
		libsimodel_initialize();
	}
	
private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<adahil_interface::msg::SensorData>::SharedPtr sensor_data_publisher;
	rclcpp::Publisher<adahil_interface::msg::MavlinkSensorData>::SharedPtr mavlink_sensor_data_publisher;
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
		adahil_interface::msg::SensorData sensor_msg;
		sensor_msg.header.frame_id = "sim_model";
		sensor_msg.header.stamp = this->get_clock()->now();// builtin_interfaces::msg::Time now = this->get_clock()->now();
		
		split16bitTo8bit(libsimodel_Y.GyroSensorData[0], &sensor_msg.gyrox_h, &sensor_msg.gyrox_l);
		split16bitTo8bit(libsimodel_Y.GyroSensorData[1], &sensor_msg.gyroy_h, &sensor_msg.gyroy_l);
		split16bitTo8bit(libsimodel_Y.GyroSensorData[2], &sensor_msg.gyroz_h, &sensor_msg.gyroz_l);

		split16bitTo8bit(libsimodel_Y.AccelSensorData[0], &sensor_msg.accelx_h, &sensor_msg.accelx_l);
		split16bitTo8bit(libsimodel_Y.AccelSensorData[1], &sensor_msg.accely_h, &sensor_msg.accely_l);
		split16bitTo8bit(libsimodel_Y.AccelSensorData[2], &sensor_msg.accelz_h, &sensor_msg.accelz_l);

		split16bitTo8bit(libsimodel_Y.TempData, &sensor_msg.temp_h, &sensor_msg.temp_l);

		sensor_data_publisher->publish(sensor_msg);

		adahil_interface::msg::MavlinkSensorData mavlink_sensor_msg;
		mavlink_sensor_msg.header.frame_id = "sim_model";
		mavlink_sensor_msg.header.stamp = this->get_clock()->now();
		
		mavlink_sensor_msg.set__gyro({libsimodel_Y.MavLinkSensorData.xgyro, libsimodel_Y.MavLinkSensorData.ygyro, libsimodel_Y.MavLinkSensorData.zgyro});
		mavlink_sensor_msg.set__accel({libsimodel_Y.MavLinkSensorData.xacc,libsimodel_Y.MavLinkSensorData.yacc,libsimodel_Y.MavLinkSensorData.zacc});
		mavlink_sensor_msg.set__mag({libsimodel_Y.MavLinkSensorData.xmag, libsimodel_Y.MavLinkSensorData.ymag, libsimodel_Y.MavLinkSensorData.zmag});
		mavlink_sensor_msg.set__abs_pressure(libsimodel_Y.MavLinkSensorData.abs_pressure);
		mavlink_sensor_msg.set__temperature(libsimodel_Y.MavLinkSensorData.temperature);
		mavlink_sensor_data_publisher->publish(mavlink_sensor_msg);
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
