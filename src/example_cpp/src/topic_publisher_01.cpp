#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sys/time.h>
#include "posix/spi.h"
#include <string.h>


extern "C"{
	// #include "fadd.h"
	#include "libsimodel.h"
}


#define spi1_device		"/dev/spidev1.0"

class TopicPublisher01 : public rclcpp::Node, public SPI
{
public:
    // 构造函数, 有一个参数为节点名称
    TopicPublisher01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
		
		//初始化SPI，并进行测试
		//initial spi0
		int ret = SPI::init(spi1_device, SPIDEV_MODE3, 10000000);//SPIDEV_MODE3
		if (ret != PX4_OK) {
			printf("SPI::init failed (%i)", ret);
		}
		// RegisterWrite(0x01, 0x88);
		// RCLCPP_INFO(this->get_logger(), "write 0x01 = %x",RegisterRead(0x01));
		icm20689reg icmreg;
		SensorRegBurstRead(&icmreg);
		for (int i = 0; i < 130; ++i) {
			RCLCPP_INFO(this->get_logger(), "reg %x(%d) data read is: %x\n", i, i, icmreg.reg_data[i]);
		}
		// libsimodel_initialize();
		// RegisterWrite(0x6b, 0x40);
		RCLCPP_INFO(this->get_logger(), "write 0x6b = %x",RegisterRead(0x6b));
		RCLCPP_INFO(this->get_logger(), "Read 0x75(117) = %x",RegisterRead(0x75));
		
		// 创建发布者
        command_publisher_ = this->create_publisher<std_msgs::msg::String>("command", 10);
		// 创建定时器，xxxms为周期，定时发布
		//milliseconds表示毫秒  microseconds表示微妙
        timer_ = this->create_wall_timer(std::chrono::microseconds(1000), std::bind(&TopicPublisher01::timer_callback, this));
    }

private:
	struct timespec tBeginTime, tEndTime;
	struct timespec tBeginTime_spi, tEndTime_spi;
	struct icm20689reg {
		uint8_t cmd{0x80};
		uint8_t reg_data[130] {0,};
	};
	// Transfer data
	// struct WriteBuffer {
	// 	uint8_t cmd{0x00 | 0x00};
	// 	uint8_t Gyro_data[6] {0,};
	// 	uint8_t Temp_data[2] {0,};
	// 	uint8_t Accel_data[6] {0,};
	// 	uint8_t Mag_data[6] {0,};
	// 	uint8_t Pressure_data[6] {0,};
	// };
	// struct ReadBuffer {
	// 	uint8_t cmd{0x00 | 0x80};
	// 	uint8_t Gyro_data[6] {0,};
	// 	uint8_t Temp_data[2] {0,};
	// 	uint8_t Accel_data[6] {0,};
	// 	uint8_t Mag_data[6] {0,};
	// 	uint8_t Pressure_data[6] {0,};
	// };
	void timer_callback()
    {
		icm20689reg icmreg;
		SensorRegBurstRead(&icmreg);
		//得到当前时间，然后与之前的时间计算插值
		// clock_gettime(CLOCK_MONOTONIC, &tEndTime);
		// double fCostTime = (tEndTime.tv_sec-tBeginTime.tv_sec)*1000000 + (double)(tEndTime.tv_nsec-tBeginTime.tv_nsec)/1000;
		// memcpy(&tBeginTime,&tEndTime,sizeof(tBeginTime));
		// printf("[clock_gettime] Cost Time = %.3fusec\n", fCostTime);
		
		// libsimodel_U.inPWMs[0] = 0;
		// libsimodel_step();
		
		// WriteBuffer buffer{};
		// buffer.Accel_data[0] = 1;
		// buffer.Accel_data[1] = 2;
		// buffer.Accel_data[2] = 3;
		// buffer.Accel_data[3] = 4;
		// buffer.Accel_data[4] = 5;
		// buffer.Accel_data[5] = 6;
		// buffer.Temp_data[0] = 7;
		// buffer.Temp_data[1] = 8;
		// buffer.Gyro_data[0] = 9;
		// buffer.Gyro_data[1] = 10;
		// buffer.Gyro_data[2] = 11;
		// buffer.Gyro_data[3] = 12;
		// buffer.Gyro_data[4] = 13;
		// buffer.Gyro_data[5] = 14;
		
		// clock_gettime(CLOCK_MONOTONIC, &tBeginTime_spi);
		// SensorDataBurstWrite(buffer);
		// RegisterWrite(0x00, 0x05);
		// libsimodel_step();
		// clock_gettime(CLOCK_MONOTONIC, &tEndTime_spi);
		// double fCostTime_spi = (tEndTime_spi.tv_sec-tBeginTime_spi.tv_sec)*1000000 + (double)(tEndTime_spi.tv_nsec-tBeginTime_spi.tv_nsec)/1000;
		// printf("[clock_gettime_spi] Cost Time = %.3fusec\n", fCostTime_spi);

		// ReadBuffer rbuffer{};
		// SensorDataBurstRead(rbuffer);
		// printf("read data is %d\n", rbuffer.Accel_data[0]);
		// uint8_t reg_data = RegisterRead(0x01);
		// printf("data read is: %d\n", reg_data);

        /* 创建消息 */ 
        // std_msgs::msg::String message;
        // message.data = "forward";
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // command_publisher_->publish(message);
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

	// uint8_t SensorDataBurstWrite(BurstBuffer *buffer)
	// {
	// 	//设置为写命令
	// 	buffer->cmd = 0x00;
	// 	return transfer((uint8_t *)buffer, (uint8_t *)buffer, sizeof(BurstBuffer));
	// }
	// uint8_t SensorDataBurstRead(BurstBuffer *buffer)
	// {
	// 	//设置为读命令
	// 	buffer->cmd = 0x80;
	// 	return transfer((uint8_t *)buffer, (uint8_t *)buffer, sizeof(BurstBuffer));
	// }
	uint8_t SensorRegBurstRead(icm20689reg *buffer)
	{
		//设置为读命令
		buffer->cmd = 0x80;
		return transfer((uint8_t *)buffer, (uint8_t *)buffer, sizeof(icm20689reg));
	}

    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
	// 声明话题发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<TopicPublisher01>("topic_publisher_01");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
