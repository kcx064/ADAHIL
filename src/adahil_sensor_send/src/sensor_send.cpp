#include <cstdio>
#include <random>
#include <ctime>
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
			RCLCPP_INFO(this->get_logger(), "[sensor send] node is running: %s.", name.c_str());

			// 初始化SPI
			int ret = SPI::init(spi0_device, SPIDEV_MODE3, 16000000);//SPIDEV_MODE3
			if (ret != 0) {
				// printf("SPI::init faile d (%i)", ret);
				RCLCPP_ERROR(this->get_logger(), "SPI::init failed (%i)", ret);
			}else{
				RCLCPP_INFO(this->get_logger(), "SPI::init success.");
			}

			//测试SPI到FPGA的读写正确率
			RCLCPP_INFO(this->get_logger(), "start test.");
			uint16_t success_test_count = 0;
			BurstBuffer bufferSend_test, bufferRecv_test;
			uint8_t test_buffer[14];
			for (int i = 0; i < 1000; i++){
				gentestbuffer(test_buffer, 14);
				bufferSend_test.Gyro_data[0] = test_buffer[0];
				bufferSend_test.Gyro_data[1] = test_buffer[1];
				bufferSend_test.Gyro_data[2] = test_buffer[2];
				bufferSend_test.Gyro_data[3] = test_buffer[3];
				bufferSend_test.Gyro_data[4] = test_buffer[4];
				bufferSend_test.Gyro_data[5] = test_buffer[5];
				bufferSend_test.Temp_data[0] = test_buffer[6];
				bufferSend_test.Temp_data[1] = test_buffer[7];
				bufferSend_test.Accel_data[0] = test_buffer[8];
				bufferSend_test.Accel_data[1] = test_buffer[9];
				bufferSend_test.Accel_data[2] = test_buffer[10];
				bufferSend_test.Accel_data[3] = test_buffer[11];
				bufferSend_test.Accel_data[4] = test_buffer[12];
				bufferSend_test.Accel_data[5] = test_buffer[13];			
				SensorDataBurstWrite(&bufferSend_test);
				usleep(1000);
				SensorDataBurstRead(&bufferRecv_test);
				usleep(1000);
				if (bufferRecv_test.Gyro_data[0] == test_buffer[0]
				&&bufferRecv_test.Gyro_data[1] == test_buffer[1]
				&&bufferRecv_test.Gyro_data[2] == test_buffer[2]
				&&bufferRecv_test.Gyro_data[3] == test_buffer[3]
				&&bufferRecv_test.Gyro_data[4] == test_buffer[4]
				&&bufferRecv_test.Gyro_data[5] == test_buffer[5]
				&&bufferRecv_test.Temp_data[0] == test_buffer[6]
				&&bufferRecv_test.Temp_data[1] == test_buffer[7]
				&&bufferRecv_test.Accel_data[0] == test_buffer[8]
				&&bufferRecv_test.Accel_data[1] == test_buffer[9]
				&&bufferRecv_test.Accel_data[2] == test_buffer[10]
				&&bufferRecv_test.Accel_data[3] == test_buffer[11]
				&&bufferRecv_test.Accel_data[4] == test_buffer[12]
				&&bufferRecv_test.Accel_data[5] == test_buffer[13])
				{
					success_test_count=success_test_count+1;
					// RCLCPP_INFO(this->get_logger(), "%d th: success test count: %d", i, success_test_count);
				}
				else{
					RCLCPP_INFO(this->get_logger(), "%d th: fail test count: %d", i, success_test_count);
					
					RCLCPP_INFO(this->get_logger(), "Gyro_data recv: %x, %x, %x, %x, %x, %x", bufferRecv_test.Gyro_data[0],bufferRecv_test.Gyro_data[1],bufferRecv_test.Gyro_data[2],bufferRecv_test.Gyro_data[3],bufferRecv_test.Gyro_data[4],bufferRecv_test.Gyro_data[5]);
					RCLCPP_INFO(this->get_logger(), "Gyro_data send: %x, %x, %x, %x, %x, %x", test_buffer[0],test_buffer[0],test_buffer[2],test_buffer[3],test_buffer[4],test_buffer[5]);

					RCLCPP_INFO(this->get_logger(), "Temp_data recv: %x, %x", bufferRecv_test.Temp_data[0],bufferRecv_test.Temp_data[1]);
					RCLCPP_INFO(this->get_logger(), "Temp_data send: %x, %x", test_buffer[6],test_buffer[7]);

					RCLCPP_INFO(this->get_logger(), "Accel_data recv: %x, %x, %x, %x, %x, %x", bufferRecv_test.Accel_data[0],bufferRecv_test.Accel_data[1],bufferRecv_test.Accel_data[2],bufferRecv_test.Accel_data[3],bufferRecv_test.Accel_data[4],bufferRecv_test.Accel_data[5]);
					RCLCPP_INFO(this->get_logger(), "Accel_data send: %x, %x, %x, %x, %x, %x", test_buffer[8],test_buffer[9],test_buffer[10],test_buffer[11],test_buffer[12],test_buffer[13]);
					
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
		void gentestbuffer(uint8_t *buffer, uint8_t length)
		{
			builtin_interfaces::msg::Time now = this->get_clock()->now();
			std::default_random_engine generator;
			generator.seed(now.nanosec);
			std::uniform_int_distribution<uint8_t> distribution(1, 100);
			for(uint8_t i = 0; i < length; i++){
				buffer[i] = distribution(generator);
			}
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
