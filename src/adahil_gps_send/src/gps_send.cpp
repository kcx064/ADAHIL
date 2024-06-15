#include <cstdio>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <iostream>
#include <ostream>
#include "adahil_interface/msg/gps_data.hpp"
#include "rclcpp/rclcpp.hpp"

#define UBX_SYNC1             0xB5
#define UBX_SYNC2             0x62

class GpsSend : public rclcpp::Node
{
	public:
		int _serial_fd = -1;
		GpsSend(std::string name) : Node(name)
		{
			RCLCPP_INFO(this->get_logger(), "Node is running: %s.", name.c_str());

			// 打开串口
			_serial_fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY);
			if (_serial_fd == -1){
				RCLCPP_ERROR(this->get_logger(), "UART open Error, _serial_fd: %d.", _serial_fd);
			}else{
				RCLCPP_INFO(this->get_logger(), "UART open Success, _serial_fd: %d.", _serial_fd);
			}
			struct termios opt;
			// 配置串口
			//清空串口接收缓冲区
			tcflush(_serial_fd, TCIOFLUSH);
			// 获取串口参数opt
			tcgetattr(_serial_fd, &opt);
			//设置串口输出波特率
			cfsetospeed(&opt, B115200);
			//设置串口输入波特率
			cfsetispeed(&opt, B115200);
			//设置数据位数 8位数据位
			opt.c_cflag &= ~CSIZE;
			opt.c_cflag |= CS8;
			//校验位 无校验位
			opt.c_cflag &= ~PARENB;
			opt.c_iflag &= ~INPCK;
			//设置停止位  1位停止位
			opt.c_cflag &= ~CSTOPB;
			opt.c_cflag |= CLOCAL | CREAD;
			opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
			opt.c_oflag &= ~OPOST;
			opt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
			//
			opt.c_cc[VTIME]=11;
			opt.c_cc[VMIN]=0;
			//更新配置
			tcsetattr(_serial_fd, TCSANOW, &opt);
			//第四部分代码/
			tcflush(_serial_fd,TCIOFLUSH);

			// 初始化解码状态机
			decodeInit();

			gps_sub = this->create_subscription<adahil_interface::msg::GPSData>("gps_data", 5, std::bind(&GpsSend::gps_callback, this, std::placeholders::_1));
			
		}

		void gps_callback(const adahil_interface::msg::GPSData::SharedPtr msg)
		{
			// RCLCPP_INFO(this->get_logger(), "GPS Data: %f, %f.", msg->lon, msg->lat);
			int err = 0, ret = 0;
			int bytes_available = 0;
			
			// 获取串口数据大小
			err = ioctl(_serial_fd, FIONREAD, (unsigned long)&bytes_available);
			if (err < 0)
			{
				RCLCPP_WARN(this->get_logger(), "FIONREAD error: %d.", err);
			}

			// 读取串口数据
			if (bytes_available > 0)
			{
				RCLCPP_INFO(this->get_logger(), "bytes_available: %d.", bytes_available);
				char buffer[256];
				ret = read(_serial_fd, buffer, bytes_available);
				if (ret < 0)
				{
					std::cerr << "Error " << errno << " from read: " << strerror(errno) << std::endl;
				}
				else
				{
					for (int i = 0; i < bytes_available; i++)
					{
						parseChar(buffer[i]);
					}
				}
			}
		}

		void parseChar(uint8_t byte_data){
			int ret = 0;
			switch (_decode_state)
			{
			case UBX_DECODE_SYNC1:
				if(byte_data == UBX_SYNC1){
					_decode_state = UBX_DECODE_SYNC2;
					RCLCPP_INFO(this->get_logger(), "UBX_DECODE_SYNC1 Success, next state is UBX_DECODE_SYNC2");
				}
				break;

			case UBX_DECODE_SYNC2:
				if(byte_data == UBX_SYNC2){
					_decode_state = UBX_DECODE_CLASS;
					RCLCPP_INFO(this->get_logger(), "UBX_DECODE_SYNC2 Success, next state is UBX_DECODE_CLASS");
				}else{
					decodeInit();
				}
				break;

			case UBX_DECODE_CLASS:
				addByteToChecksum(byte_data);
				_decode_state = UBX_DECODE_ID;
				RCLCPP_INFO(this->get_logger(), "UBX_CLASS ID is 0x%x", byte_data);
				break;

			case UBX_DECODE_ID:
				addByteToChecksum(byte_data);
				_decode_state = UBX_DECODE_LENGTH1;
				RCLCPP_INFO(this->get_logger(), "UBX_MSG ID is 0x%x", byte_data);
				break;

			case UBX_DECODE_LENGTH1:
				addByteToChecksum(byte_data);
				_rx_payload_length |= byte_data;
				_decode_state = UBX_DECODE_LENGTH2;	
				break;

			case UBX_DECODE_LENGTH2:
				addByteToChecksum(byte_data);
				_rx_payload_length |= (byte_data << 8);
				RCLCPP_INFO(this->get_logger(), "UBX_MSG LENGTH is %d", _rx_payload_length);
				_decode_state = UBX_DECODE_PAYLOAD;
				break;

			case UBX_DECODE_PAYLOAD:
				addByteToChecksum(byte_data);
				ret = payloadRxAdd(byte_data);

				if(ret>0){
					_decode_state = UBX_DECODE_CHKSUM1;
				}else{
					// expecting more payload, stay in state UBX_DECODE_PAYLOAD
				}
				ret = 0;
				break;

			case UBX_DECODE_CHKSUM1:
				if (_rx_ck_a != byte_data)
				{
					RCLCPP_INFO(this->get_logger(), "UBX_DECODE_CHKSUM1 Error, ck_a is %d", _rx_ck_a);
					decodeInit();
				}else{
					RCLCPP_INFO(this->get_logger(), "UBX_DECODE_CHKSUM1 Success");
					_decode_state = UBX_DECODE_CHKSUM2;
				}
				break;

			case UBX_DECODE_CHKSUM2:
				if (_rx_ck_b != byte_data)
				{
					RCLCPP_INFO(this->get_logger(), "UBX_DECODE_CHKSUM2 Error, ck_b is %d", _rx_ck_b);
				}else{
					RCLCPP_INFO(this->get_logger(), "UBX_DECODE_CHKSUM2 Success");
				}
				decodeInit();
				break;

			default:
				break;
			}
		}

		void decodeInit(){
			_decode_state = UBX_DECODE_SYNC1;
			_rx_ck_a = 0;
			_rx_ck_b = 0;
			_rx_payload_length = 0;
			_rx_payload_index = 0;
		}

		int payloadRxAdd(const uint8_t byte_data){
			int ret = 0;
			uint8_t *payload_buf = (uint8_t *)_buf;//临时，后面替换为联合体变量定义，此时需要改为&_buf
			payload_buf[_rx_payload_index] = byte_data;
			_rx_payload_index++;

			if(_rx_payload_index>=_rx_payload_length){
				//数据处理完毕
				ret = 1;
			}else{
				//数据处理中
				ret = 0;
			}
			return ret;
		}

		void addByteToChecksum(const uint8_t byte_data)
		{
			_rx_ck_a = _rx_ck_a + byte_data;
			_rx_ck_b = _rx_ck_b + _rx_ck_a;
		}

	private:
		rclcpp::Subscription<adahil_interface::msg::GPSData>::SharedPtr gps_sub;

		typedef enum {
			UBX_DECODE_SYNC1 = 0,
			UBX_DECODE_SYNC2,
			UBX_DECODE_CLASS,
			UBX_DECODE_ID,
			UBX_DECODE_LENGTH1,
			UBX_DECODE_LENGTH2,
			UBX_DECODE_PAYLOAD,
			UBX_DECODE_CHKSUM1,
			UBX_DECODE_CHKSUM2,
		} ubx_decode_state_t;

		ubx_decode_state_t _decode_state;
		uint8_t _rx_ck_a{0};
		uint8_t _rx_ck_b{0};
		uint16_t _rx_payload_length{0};
		uint16_t _rx_payload_index{0};
		uint8_t   _buf[256];//临时，后面替换为联合体变量定义
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<GpsSend>("gps_send");
	rclcpp::spin(node);
	rclcpp::shutdown();
	close(node->_serial_fd);
  	return 0;
}
