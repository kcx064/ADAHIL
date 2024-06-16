#include <cstdio>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <iostream>
#include <ostream>
#include "adahil_interface/msg/gps_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ubx.h"

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
						parseChar(buffer[i]);//逐字节进行处理
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
				addByteToChecksum_rx(byte_data);
				_rx_msg = byte_data;
				_decode_state = UBX_DECODE_ID;
				RCLCPP_INFO(this->get_logger(), "UBX_CLASS ID is 0x%x", byte_data);
				break;

			case UBX_DECODE_ID:
				addByteToChecksum_rx(byte_data);
				_rx_msg |= byte_data << 8;
				_decode_state = UBX_DECODE_LENGTH1;
				RCLCPP_INFO(this->get_logger(), "UBX_MSG ID is 0x%x", byte_data);
				break;

			case UBX_DECODE_LENGTH1:
				addByteToChecksum_rx(byte_data);
				_rx_payload_length |= byte_data;
				_decode_state = UBX_DECODE_LENGTH2;	
				break;

			case UBX_DECODE_LENGTH2:
				addByteToChecksum_rx(byte_data);
				_rx_payload_length |= (byte_data << 8);
				RCLCPP_INFO(this->get_logger(), "UBX_MSG LENGTH is %d", _rx_payload_length);
				if(_rx_payload_length>0)
				{
					_decode_state = UBX_DECODE_PAYLOAD;
				}
				else
				{
					_decode_state = UBX_DECODE_CHKSUM1;
				}
				break;

			case UBX_DECODE_PAYLOAD:
				addByteToChecksum_rx(byte_data);
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
					RCLCPP_INFO(this->get_logger(), "UBX_DECODE_CHKSUM1 Error, ck_a is %d, byte_data is %d", _rx_ck_a, byte_data);
					decodeInit();
				}else{
					RCLCPP_INFO(this->get_logger(), "UBX_DECODE_CHKSUM1 Success");
					_decode_state = UBX_DECODE_CHKSUM2;
				}
				break;

			case UBX_DECODE_CHKSUM2:
				if (_rx_ck_b != byte_data)
				{
					RCLCPP_INFO(this->get_logger(), "UBX_DECODE_CHKSUM2 Error, ck_b is %d, byte_data is %d", _rx_ck_b, byte_data);
				}else{
					RCLCPP_INFO(this->get_logger(), "UBX_DECODE_CHKSUM2 Success");
					decode_payload(_rx_msg, (uint8_t*)&_buf);
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

		/**
		 * @brief 向接收校验和添加字节
		 *
		 * 将给定的字节数据添加到接收校验和中。
		 *
		 * @param byte_data 字节数据
		 */
		void addByteToChecksum_rx(const uint8_t byte_data)
		{
			_rx_ck_a = _rx_ck_a + byte_data;
			_rx_ck_b = _rx_ck_b + _rx_ck_a;
		}

		/**
		 * @brief 将字节数据添加到发送校验和中
		 *
		 * 将给定的字节数据添加到发送校验和中，并更新校验和的值。
		 *
		 * @param byte_data 字节数据
		 */
		void addByteToChecksum_tx(const uint8_t byte_data)
		{
			_tx_ck_a = _tx_ck_a + byte_data;
			_tx_ck_b = _tx_ck_b + _tx_ck_a;
		}

		void decode_payloadInit(void)
		{
			_tx_ck_a = 0;
			_tx_ck_b = 0;
		}

		void decode_payload(uint16_t _msg, uint8_t *payload)
		{
			switch (_msg)
			{
				case UBX_MSG_CFG_VALSET:
					ubx_tx_ack_nak_t ubx_tx_ack;
					ubx_tx_ack.msg_s.clsID = UBX_CLASS_ACK;
					ubx_tx_ack.msg_s.msgID = UBX_ID_ACK_ACK;
					ubx_tx_ack.msg_s.length = 2;
					ubx_tx_ack.msg_s.payload[0] = uint8_t(UBX_MSG_CFG_VALSET & 0x0F);
					ubx_tx_ack.msg_s.payload[1] = uint8_t(UBX_MSG_CFG_VALSET >> 8);
					for(size_t i=0; i < sizeof(ubx_tx_ack); i++){
						addByteToChecksum_tx(ubx_tx_ack.msg_buf[i]);
					}

					{
						uint8_t sync_buf[] = {UBX_SYNC1, UBX_SYNC2};
						write(_serial_fd, sync_buf, sizeof(sync_buf));
					}

					write(_serial_fd, &ubx_tx_ack, sizeof(ubx_tx_ack));

					{
						uint8_t tx_checksum[] = {_tx_ck_a, _tx_ck_b};
						write(_serial_fd, tx_checksum, sizeof(tx_checksum));
					}

					decode_payloadInit();
					RCLCPP_INFO(this->get_logger(), "UBX_MSG_CFG_VALSET Success");
				break;

				case UBX_MSG_CFG_PRT:
					ubx_tx_ack_nak_t ubx_tx_nak;
					ubx_tx_nak.msg_s.clsID = UBX_CLASS_ACK;
					ubx_tx_nak.msg_s.msgID = UBX_ID_ACK_NAK;
					ubx_tx_nak.msg_s.length = 2;
					ubx_tx_nak.msg_s.payload[0] = uint8_t(UBX_MSG_CFG_PRT & 0x0F);
					ubx_tx_nak.msg_s.payload[1] = uint8_t(UBX_MSG_CFG_PRT >> 8);
					for(size_t i=0; i < sizeof(ubx_tx_nak); i++){
						addByteToChecksum_tx(ubx_tx_nak.msg_buf[i]);
					}

					{
						uint8_t sync_buf[] = {UBX_SYNC1, UBX_SYNC2};
						write(_serial_fd, sync_buf, sizeof(sync_buf));
					}

					write(_serial_fd, &ubx_tx_nak, sizeof(ubx_tx_nak));

					{
						uint8_t tx_checksum[] = {_tx_ck_a, _tx_ck_b};
						write(_serial_fd, tx_checksum, sizeof(tx_checksum));
					}

					decode_payloadInit();
					RCLCPP_INFO(this->get_logger(), "UBX_MSG_CFG_PRT Success");
				break;
			}
		}

	private:
		rclcpp::Subscription<adahil_interface::msg::GPSData>::SharedPtr gps_sub;

		enum ubx_decode_state
		{
			UBX_DECODE_SYNC1 = 0,
			UBX_DECODE_SYNC2,
			UBX_DECODE_CLASS,
			UBX_DECODE_ID,
			UBX_DECODE_LENGTH1,
			UBX_DECODE_LENGTH2,
			UBX_DECODE_PAYLOAD,
			UBX_DECODE_CHKSUM1,
			UBX_DECODE_CHKSUM2,
		}; 
		typedef enum ubx_decode_state ubx_decode_state_t;

		ubx_decode_state_t _decode_state;
		uint8_t _rx_ck_a{0};
		uint8_t _rx_ck_b{0};
		uint16_t _rx_payload_length{0};
		uint16_t _rx_payload_index{0};
		uint16_t _rx_msg{};
		uint8_t _buf[256];//临时，后面替换为联合体变量定义

		uint8_t _tx_ck_a{0};
		uint8_t _tx_ck_b{0};
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
