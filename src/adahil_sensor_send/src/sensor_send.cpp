#include <cstdio>
#include "spi.h"
#include "rclcpp/rclcpp.hpp"
#include "adahil_interface/msg/sensor_data.hpp"

#define spi0_device		"/dev/spidev0.0"
#define spi1_device		"/dev/spidev1.0"

class SensorSend : public rclcpp::Node, public SPI
{
	public:
		//构造函数
		SensorSend(std::string name) : Node(name)
		{
			RCLCPP_INFO(this->get_logger(), "node is running: %s.", name.c_str());

			//初始化spi
			int ret = SPI::init(spi0_device, SPIDEV_MODE3, 15000000);//SPIDEV_MODE3
			if (ret != 0) {
				// printf("SPI::init failed (%i)", ret);
				RCLCPP_ERROR(this->get_logger(), "SPI::init failed (%i)", ret);
			}else{
				RCLCPP_INFO(this->get_logger(), "SPI::init success.");
			}
			BurstBuffer bufferSend_test, bufferSend_compared;
			bufferSend_compared.Gyro_data[0] = 0x01;
			bufferSend_compared.Gyro_data[1] = 0x02;
			bufferSend_compared.Gyro_data[2] = 0x03;
			bufferSend_compared.Gyro_data[3] = 0x04;
			bufferSend_compared.Gyro_data[4] = 0x05;
			bufferSend_compared.Gyro_data[5] = 0x06;
			bufferSend_compared.Temp_data[0] = 0x07;
			bufferSend_compared.Temp_data[1] = 0x08;
			bufferSend_compared.Accel_data[0] = 0x09;
			bufferSend_compared.Accel_data[1] = 0x0a;
			bufferSend_compared.Accel_data[2] = 0x0b;
			bufferSend_compared.Accel_data[3] = 0x0c;
			bufferSend_compared.Accel_data[4] = 0x0d;
			bufferSend_compared.Accel_data[5] = 0x0e;

			bufferSend_test.Gyro_data[0] = 0x01;
			bufferSend_test.Gyro_data[1] = 0x02;
			bufferSend_test.Gyro_data[2] = 0x03;
			bufferSend_test.Gyro_data[3] = 0x04;
			bufferSend_test.Gyro_data[4] = 0x05;
			bufferSend_test.Gyro_data[5] = 0x06;
			bufferSend_test.Temp_data[0] = 0x07;
			bufferSend_test.Temp_data[1] = 0x08;
			bufferSend_test.Accel_data[0] = 0x09;
			bufferSend_test.Accel_data[1] = 0x0a;
			bufferSend_test.Accel_data[2] = 0x0b;
			bufferSend_test.Accel_data[3] = 0x0c;
			bufferSend_test.Accel_data[4] = 0x0d;
			bufferSend_test.Accel_data[5] = 0x0e;
			SensorDataBurstWrite(&bufferSend_test);
			usleep(1000*1000);
			uint16_t success_test_count = 0;
			RCLCPP_INFO(this->get_logger(), "start test.");
			for (int i = 0; i < 1000; i++){
				BurstBuffer bufferRecv_test;
				SensorDataBurstRead(&bufferRecv_test);
				usleep(1*1000);
				if (bufferRecv_test.Gyro_data[0] == bufferSend_compared.Gyro_data[0]&&bufferRecv_test.Gyro_data[1] == bufferSend_compared.Gyro_data[1]&&bufferRecv_test.Gyro_data[2] == bufferSend_compared.Gyro_data[2]&&bufferRecv_test.Gyro_data[3] == bufferSend_compared.Gyro_data[3]&&bufferRecv_test.Gyro_data[4] == bufferSend_compared.Gyro_data[4]&&bufferRecv_test.Gyro_data[5] == bufferSend_compared.Gyro_data[5]&&bufferRecv_test.Accel_data[0] == bufferSend_compared.Accel_data[0]&&bufferRecv_test.Accel_data[1] == bufferSend_compared.Accel_data[1]&&bufferRecv_test.Accel_data[2] == bufferSend_compared.Accel_data[2]&&bufferRecv_test.Accel_data[3] == bufferSend_compared.Accel_data[3]&&bufferRecv_test.Accel_data[4] == bufferSend_compared.Accel_data[4]&&bufferRecv_test.Accel_data[5] == bufferSend_compared.Accel_data[5]&&bufferRecv_test.Temp_data[0] == bufferSend_compared.Temp_data[0]&&bufferRecv_test.Temp_data[1] == bufferSend_compared.Temp_data[1]
				){
					success_test_count=success_test_count+1;
					// RCLCPP_INFO(this->get_logger(), "%d th: success test count: %d", i, success_test_count);
				}
				else{
					RCLCPP_INFO(this->get_logger(), "%d th: fail test count: %d", i, success_test_count);
					
					// RCLCPP_INFO(this->get_logger(), "Gyro_data: %x, %x, %x, %x, %x, %x", bufferRecv_test.Gyro_data[0],bufferRecv_test.Gyro_data[1],bufferRecv_test.Gyro_data[2],bufferRecv_test.Gyro_data[3],bufferRecv_test.Gyro_data[4],bufferRecv_test.Gyro_data[5]);
					// RCLCPP_INFO(this->get_logger(), "Gyro_data: %x, %x, %x, %x, %x, %x", bufferSend_compared.Gyro_data[0],bufferSend_compared.Gyro_data[1],bufferSend_compared.Gyro_data[2],bufferSend_compared.Gyro_data[3],bufferSend_compared.Gyro_data[4],bufferSend_compared.Gyro_data[5]);

					// RCLCPP_INFO(this->get_logger(), "Temp_data: %x, %x", bufferRecv_test.Temp_data[0],bufferRecv_test.Temp_data[1]);
					// RCLCPP_INFO(this->get_logger(), "Temp_data: %x, %x", bufferSend_compared.Temp_data[0],bufferSend_compared.Temp_data[1]);

					// RCLCPP_INFO(this->get_logger(), "Accel_data: %x, %x, %x, %x, %x, %x", bufferRecv_test.Accel_data[0],bufferRecv_test.Accel_data[1],bufferRecv_test.Accel_data[2],bufferRecv_test.Accel_data[3],bufferRecv_test.Accel_data[4],bufferRecv_test.Accel_data[5]);
					// RCLCPP_INFO(this->get_logger(), "Accel_data: %x, %x, %x, %x, %x, %x", bufferSend_compared.Accel_data[0],bufferSend_compared.Accel_data[1],bufferSend_compared.Accel_data[2],bufferSend_compared.Accel_data[3],bufferSend_compared.Accel_data[4],bufferSend_compared.Accel_data[5]);
					
				}
							
			}
			RCLCPP_INFO(this->get_logger(), "success test count: %d", success_test_count);
			
			//创建订阅者
			sensor_data_sub = this->create_subscription<adahil_interface::msg::SensorData>("sensor_data", 10, std::bind(&SensorSend::sub_sensor_data_callback, this, std::placeholders::_1) );
		}
	private:
		// 声明订阅者
		rclcpp::Subscription<adahil_interface::msg::SensorData>::SharedPtr sensor_data_sub;
		// Transfer data
		struct BurstBuffer {
			uint8_t cmd{0x00};
			uint8_t Gyro_data[6] {0,};
			uint8_t Temp_data[2] {0,};
			uint8_t Accel_data[6] {0,};
			uint8_t Mag_data[6] {0,};
			uint8_t Pressure_data[6] {0,};
		};

		void sub_sensor_data_callback(const adahil_interface::msg::SensorData::SharedPtr msg)
		{
			// RCLCPP_DEBUG(this->get_logger(), "sensor data: %x", msg->gyrox_h);
			BurstBuffer bufferSend;
			bufferSend.Gyro_data[0] = msg->gyrox_h;
			bufferSend.Gyro_data[1] = msg->gyrox_l;
			bufferSend.Gyro_data[2] = msg->gyroy_h;
			bufferSend.Gyro_data[3] = msg->gyroy_l;
			bufferSend.Gyro_data[4] = msg->gyroz_h;
			bufferSend.Gyro_data[5] = msg->gyroz_l;
			bufferSend.Temp_data[0] = msg->temp_h;
			bufferSend.Temp_data[1] = msg->temp_l;
			bufferSend.Accel_data[0] = msg->accelx_h;
			bufferSend.Accel_data[1] = msg->accelx_l;
			bufferSend.Accel_data[2] = msg->accely_h;
			bufferSend.Accel_data[3] = msg->accely_l;
			bufferSend.Accel_data[4] = msg->accelz_h;
			bufferSend.Accel_data[5] = msg->accelz_l;
			SensorDataBurstWrite(&bufferSend);
		}
		template <typename T>
		uint8_t RegisterRead(T reg)
		{
			uint8_t cmd[2] {};
			cmd[0] = static_cast<uint8_t>(reg) | 0x80;
			transfer(cmd, cmd, sizeof(cmd));
			return cmd[1];
		}

		template <typename T>
		void RegisterWrite(T reg, uint8_t value)
		{
			uint8_t cmd[2] { (uint8_t)reg, value };
			transfer(cmd, cmd, sizeof(cmd));
		}

		uint8_t SensorDataBurstWrite(BurstBuffer *buffer)
		{
			//设置为写命令
			// buffer.cmd = 0x00;
			// return transfer((uint8_t *)&buffer, (uint8_t *)&buffer, sizeof(buffer));
			buffer->cmd = 0x00;
			return transfer((uint8_t *)buffer, (uint8_t *)buffer, sizeof(BurstBuffer));
		}
		uint8_t SensorDataBurstRead(BurstBuffer *buffer)
		{
			//设置为读命令
			// BurstBuffer bufferRecvCmd;
			// bufferRecvCmd.cmd = 0x80;
			// return transfer((uint8_t *)&bufferRecvCmd, (uint8_t *)buffer, sizeof(bufferRecvCmd));
			buffer->cmd = 0x80;
			return transfer((uint8_t *)buffer, (uint8_t *)buffer, sizeof(BurstBuffer));
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
