#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "posix/spi.h"

#define spi0_device		"/dev/spidev0.0"

class TopicPublisher01 : public rclcpp::Node, public SPI
{
public:
    // 构造函数, 有一个参数为节点名称
    TopicPublisher01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
		
		//初始化SPI，并进行测试
		//initial spi0
		int ret = SPI::init(spi0_device, SPIDEV_MODE3, 10000000);
		if (ret != PX4_OK) {
			printf("SPI::init failed (%i)", ret);
		}

		uint8_t reg_data;

		RegisterWrite(0x00, 0x00);
		reg_data = RegisterRead(0x00);
		printf("data read is: %d\n", reg_data);

		RegisterWrite(0x00, 0x05);
		reg_data = RegisterRead(0x00);
		printf("data read is: %d\n", reg_data);

		reg_data = RegisterRead(0x01);
		printf("data read is: %d\n", reg_data);

		reg_data = RegisterRead(0x02);
		printf("data read is: %d\n", reg_data);

		reg_data = RegisterRead(0x03);
		printf("data read is: %d\n", reg_data);

		reg_data = RegisterRead(0x04);
		printf("data read is: %d\n", reg_data);

		reg_data = RegisterRead(0x05);
		printf("data read is: %d\n", reg_data);


		// 创建发布者
        command_publisher_ = this->create_publisher<std_msgs::msg::String>("command", 10);
		// 创建定时器，500ms为周期，定时发布
		//milliseconds表示毫秒  microseconds表示微妙
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TopicPublisher01::timer_callback, this));
    }

private:
	// Transfer data
	struct TransferBuffer {
		uint8_t cmd{0x00 | 0x00};
		uint8_t Accel_data[6] {0,};
		uint8_t Temp_data[2] {0,};
		uint8_t Gyro_data[6] {0,};
	};
	void timer_callback()
    {
		TransferBuffer buffer{};
		buffer.Accel_data[0] = 1;
		buffer.Accel_data[1] = 2;
		buffer.Accel_data[2] = 3;
		buffer.Accel_data[3] = 4;
		buffer.Accel_data[4] = 5;
		buffer.Accel_data[5] = 6;
		SensorDataBurstWrite(buffer);
        // 创建消息
        std_msgs::msg::String message;
        message.data = "forward";
        // 日志打印
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // 发布消息
        command_publisher_->publish(message);
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

	void SensorDataBurstWrite(TransferBuffer buffer)
	{
		transfer((uint8_t *)&buffer, (uint8_t *)&buffer, sizeof(buffer));
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
