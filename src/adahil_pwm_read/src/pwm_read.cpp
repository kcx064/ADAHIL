#include <cstdio>
#include <random>
#include <ctime>
#include "spi.h"
#include "rclcpp/rclcpp.hpp"
#include "adahil_interface/msg/pwm_data.hpp"

#define spi1_device		"/dev/spidev1.0"

class PWMRead : public rclcpp::Node, public SPI
{
	public:
		PWMRead(std::string name) : Node(name)
		{
			RCLCPP_INFO(this->get_logger(), "Node is running: %s.", name.c_str());

			//初始化SPI
			int ret = SPI::init(spi1_device, SPIDEV_MODE3, 16000000);//SPIDEV_MODE3
			if (ret != 0) {
				RCLCPP_ERROR(this->get_logger(), "SPI::init failed (%i)", ret);
			}else{
				RCLCPP_INFO(this->get_logger(), "SPI::init success.");
			}
			//测试SPI到FPGA的读写正确率
			RCLCPP_INFO(this->get_logger(), "start test.");

			timer_1khz = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PWMRead::timer_callback_1khz, this));

		}

	private:
		rclcpp::TimerBase::SharedPtr timer_1khz;

		#pragma pack(push, 1)
		typedef union burst_buffer
		{
			uint8_t buf[33];
			struct buf_struct 
			{
				uint8_t cmd{0x00};
				uint32_t pwm[8] {0,};
			} buf_s;
		} burst_buffer_t;
		#pragma pack(pop)

		burst_buffer_t read_buf{0,};

		//每个PWM通道由四个字节组成，高字节在前，低字节在后(其实是大端模式了，建议改成小端模式)
		uint8_t DataBurstRead(burst_buffer_t *buffer)
		{
			//设置为读命令
			buffer->buf_s.cmd = 0x80;
			return transfer((uint8_t *)buffer->buf, (uint8_t *)buffer->buf, sizeof(read_buf));
		}

		void timer_callback_1khz(){
			DataBurstRead(&read_buf);
			RCLCPP_WARN(this->get_logger(), "pwm0 = %u, pwm1 = %u,pwm2 = %u, pwm3 = %u", read_buf.buf_s.pwm[0], read_buf.buf_s.pwm[1], read_buf.buf_s.pwm[2], read_buf.buf_s.pwm[3]);
		}

};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<PWMRead>("pwm_read");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
