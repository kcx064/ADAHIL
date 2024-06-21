// #include <cstdio>
#include "adahil_interface/msg/sensor_data.hpp"
#include "adahil_interface/msg/mavlink_sensor_data.hpp"
#include "adahil_interface/msg/unreal_display_data.hpp"
#include "adahil_interface/msg/gps_data.hpp"
#include "rclcpp/rclcpp.hpp"

extern "C"{
	#include "libsimodel.h"
}

class SimModel : public rclcpp::Node {
public:
	// 构造函数,有一个参数为节点名称
	SimModel(std::string name) : Node(name) {
		RCLCPP_INFO(this->get_logger(), "Node is running %s.", name.c_str());
		sensor_data_publisher = this->create_publisher<adahil_interface::msg::SensorData>("sensor_data",5);
		mavlink_sensor_data_publisher = this->create_publisher<adahil_interface::msg::MavlinkSensorData>("mavlink_sensor_data",5);
		unreal_display_data_publisher = this->create_publisher<adahil_interface::msg::UnrealDisplayData>("unreal_display_data",5);
		gps_data_publisher = this->create_publisher<adahil_interface::msg::GPSData>("gps_data",5);
		// 创建定时器，xxxms为周期。milliseconds 表示毫秒  microseconds 表示微妙
		timer_main_pub = this->create_wall_timer(std::chrono::microseconds(500), std::bind(&SimModel::timer_callback_main, this));
		timer_unreal_data_pub = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&SimModel::timer_callback_unreal_data_pub, this));
		timer_gps_data_pub = this->create_wall_timer(std::chrono::milliseconds(125), std::bind(&SimModel::timer_callback_gps_data_pub, this));
		libsimodel_initialize();
	}
	
private:
	rclcpp::TimerBase::SharedPtr timer_main_pub;
	rclcpp::TimerBase::SharedPtr timer_unreal_data_pub;
	rclcpp::TimerBase::SharedPtr timer_gps_data_pub;
	rclcpp::Publisher<adahil_interface::msg::SensorData>::SharedPtr sensor_data_publisher;
	rclcpp::Publisher<adahil_interface::msg::MavlinkSensorData>::SharedPtr mavlink_sensor_data_publisher;
	rclcpp::Publisher<adahil_interface::msg::UnrealDisplayData>::SharedPtr unreal_display_data_publisher;
	rclcpp::Publisher<adahil_interface::msg::GPSData>::SharedPtr gps_data_publisher;
	void timer_callback_main(){
		//set input
		libsimodel_U.inPWMs[0] = 0;
		libsimodel_U.inPWMs[1] = 0;
		libsimodel_U.inPWMs[2] = 0;
		libsimodel_U.inPWMs[3] = 0;
		//simulation step
		libsimodel_step();
		//set output
		
		//pub sensor_msg
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

		split16bitTo8bit(libsimodel_Y.MagSensorData[0], &sensor_msg.magx_h, &sensor_msg.magx_l);
		split16bitTo8bit(libsimodel_Y.MagSensorData[1], &sensor_msg.magy_h, &sensor_msg.magy_l);
		split16bitTo8bit(libsimodel_Y.MagSensorData[2], &sensor_msg.magz_h, &sensor_msg.magz_l);

		split32bitTo8bit(libsimodel_Y.PressureTempData[0], &sensor_msg.baropressure_h, &sensor_msg.baropressure_m, &sensor_msg.baropressure_l);
		split32bitTo8bit(libsimodel_Y.PressureTempData[1], &sensor_msg.barotemperature_h, &sensor_msg.barotemperature_m, &sensor_msg.barotemperature_l);

		sensor_data_publisher->publish(sensor_msg);
		
		//pub mavlink_sensor_msg
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

	void timer_callback_unreal_data_pub(){
		//pub unreal_display_msg
		adahil_interface::msg::UnrealDisplayData unreal_display_msg;
		
		unreal_display_msg.header.frame_id = "sim_model";
		unreal_display_msg.header.stamp = this->get_clock()->now();
		std::array<uint8_t, 200> unrealDataArray;
		std::copy_n(reinterpret_cast<uint8_t*>(libsimodel_Y.UnRealData), 200, unrealDataArray.begin());
		unreal_display_msg.set__unrealdata(unrealDataArray);

		unreal_display_data_publisher->publish(unreal_display_msg);
	}

	void timer_callback_gps_data_pub(){
		//pub gps_data
		adahil_interface::msg::GPSData gps_data;
		gps_data.header.frame_id = "sim_model";
		gps_data.header.stamp = this->get_clock()->now();

		gps_data.fix_type = libsimodel_Y.HILGPSData.fix_type;
		gps_data.satellites_visible = libsimodel_Y.HILGPSData.satellites_visible;

		gps_data.lat = libsimodel_Y.HILGPSData.lat;
		gps_data.lon = libsimodel_Y.HILGPSData.lon;
		gps_data.alt = libsimodel_Y.HILGPSData.alt;
		
		gps_data.hacc = libsimodel_Y.HILGPSData.hAcc;
		gps_data.vacc = libsimodel_Y.HILGPSData.vAcc;
		
		gps_data.veln = libsimodel_Y.HILGPSData.velN;
		gps_data.vele = libsimodel_Y.HILGPSData.velE;
		gps_data.veld = libsimodel_Y.HILGPSData.velD;

		gps_data.gspeed = libsimodel_Y.HILGPSData.gSpeed;
		gps_data.headmot = libsimodel_Y.HILGPSData.headMot;
		gps_data.headveh = libsimodel_Y.HILGPSData.headVeh;

		gps_data_publisher->publish(gps_data);

	}

	void split16bitTo8bit(int16_t input, uint8_t *high, uint8_t *low)
	{
		*high = (input >> 8) & 0xFF;
		*low = input & 0xFF;
	}

	//将32位整数的低24位拆分为高八位 中八位和低八位
	void split32bitTo8bit(int32_t input, uint8_t *high, uint8_t *mid, uint8_t *low)
	{
		*high = (input >> 16) & 0xFF;
		*mid = (input >> 8) & 0xFF;
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
