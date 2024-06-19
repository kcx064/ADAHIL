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
			parseInit();

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
				RCLCPP_INFO(this->get_logger(), " ");
				RCLCPP_INFO(this->get_logger(), "-------------------------------------------------");
				RCLCPP_INFO(this->get_logger(), "-------------------------------------------------");
				RCLCPP_INFO(this->get_logger(), "Begin decode ubx package!");
				if(byte_data == UBX_SYNC1){
					_decode_state = UBX_DECODE_SYNC2;
					RCLCPP_INFO(this->get_logger(), "UBX_DECODE_SYNC1 Success, next state is UBX_DECODE_SYNC2");
				}else{
					RCLCPP_INFO(this->get_logger(), "byte_data is %02x", byte_data);
					parseInit();
				}
				break;

			case UBX_DECODE_SYNC2:
				if(byte_data == UBX_SYNC2){
					_decode_state = UBX_DECODE_CLASS;
					RCLCPP_INFO(this->get_logger(), "UBX_DECODE_SYNC2 Success, next state is UBX_DECODE_CLASS");
				}else{
					parseInit();
				}
				break;

			case UBX_DECODE_CLASS:
				addByteToChecksum_rx(byte_data);
				_rx_msg = byte_data;
				_decode_state = UBX_DECODE_ID;
				RCLCPP_INFO(this->get_logger(), "UBX_CLASS ID is 0x%02x", byte_data);
				break;

			case UBX_DECODE_ID:
				addByteToChecksum_rx(byte_data);
				_rx_msg |= byte_data << 8;
				_decode_state = UBX_DECODE_LENGTH1;
				RCLCPP_INFO(this->get_logger(), "UBX_MSG ID is 0x%02x", byte_data);
				break;

			case UBX_DECODE_LENGTH1:
				addByteToChecksum_rx(byte_data);
				_rx_payload_length |= byte_data;
				_decode_state = UBX_DECODE_LENGTH2;	
				break;

			case UBX_DECODE_LENGTH2:
				addByteToChecksum_rx(byte_data);
				_rx_payload_length |= (byte_data << 8);
				RCLCPP_INFO(this->get_logger(), "UBX_MSG PAYLOAD LENGTH is %d", _rx_payload_length);
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
				//该状态下，每次将解析的字节存入 _payload_buf 中
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
					parseInit();
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
					RCLCPP_INFO(this->get_logger(), "UBX_DECODE_CHKSUM2 Success, begin decode payload!");
					// 进一步地，解析消息内容
					decode_payload(_rx_msg, (uint8_t*)&_payload_buf, _rx_payload_length);
				}
				parseInit();
				break;

			default:
				break;
			}
		}

		void parseInit(){
			_decode_state = UBX_DECODE_SYNC1;
			_rx_ck_a = 0;
			_rx_ck_b = 0;
			_rx_payload_length = 0;
			_rx_payload_index = 0;
		}

		int payloadRxAdd(const uint8_t byte_data){
			int ret = 0;
			uint8_t *payload_buf = (uint8_t *)_payload_buf;//临时，后面替换为联合体变量定义，此时需要改为&_payload_buf
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

		/**
		 * @brief 解码消息内容
		 *
		 * 根据传入的消息类型，对消息payload部分进行解码，并发送相应的响应。
		 * 该函数在 parseChar() 函数运行到UBX_DECODE_CHKSUM2状态后执行一次。该状态意味着检测到了一帧完整的数据包。
		 * 
		 * @param _msg 消息类型
		 * @param payload 消息内容
		 */
		void decode_payload(uint16_t _msg, uint8_t *payload, uint16_t payload_length)
		{
			ubx_tx_ack_nak_t ubx_tx_ack;
			ubx_tx_ack_nak_t ubx_tx_nak;
			switch (_msg)
			{
				case UBX_MSG_CFG_VALSET:
					RCLCPP_INFO(this->get_logger(), "-------------------------------------------------");
					RCLCPP_INFO(this->get_logger(), "UBX_MSG_CFG_VALSET begin, payload_length is %d", payload_length);
					//解析UBX_MSG_CFG_VALSET VALSET
					for(uint16_t i=0; i < payload_length; i++){
						parse_cfg_valset_payload(payload[i], payload_length-1-i);
					}
										
					//答复ack
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

				//发送无响应消息，这里模拟ublox的新消息接口
				case UBX_MSG_CFG_PRT:
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

				case UBX_MSG_MON_VER:
					//PX4发送该消息后需要等待ACK回复，否则会返回异常。因此这里返回ACK消息。
					ubx_tx_ack.msg_s.clsID = UBX_CLASS_ACK;
					ubx_tx_ack.msg_s.msgID = UBX_ID_ACK_ACK;
					ubx_tx_ack.msg_s.length = 2;
					ubx_tx_ack.msg_s.payload[0] = uint8_t(UBX_MSG_MON_VER & 0x0F);
					ubx_tx_ack.msg_s.payload[1] = uint8_t(UBX_MSG_MON_VER >> 8);
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

					ubx_payload_mon_ver_t ubx_payload_tx_mon_ver;
					ubx_payload_tx_mon_ver.clsID = UBX_CLASS_MON;
					ubx_payload_tx_mon_ver.msgID = UBX_ID_MON_VER;
					ubx_payload_tx_mon_ver.length = 70;
					// ubx_payload_tx_mon_ver.swVersion;
					const char* hwV = "00190000";
					size_t hwVLength = std::strlen(hwV); 
					std::strncpy(reinterpret_cast<char*>(ubx_payload_tx_mon_ver.hwVersion), hwV, hwVLength);
					// ubx_payload_tx_mon_ver.extension;

					RCLCPP_INFO(this->get_logger(), "UBX_MSG_MON_VER Success");
					break;
			}
		}

		void parse_cfg_valset_payload(uint8_t bytedata, uint16_t bytedata_remain)
		{
			uint8_t is_ram{0}, is_bbr{0}, is_flash{0};
			//临时存储cfg_id中的 cfg_value_size 值
			uint32_t cfg_value_size{0};
			switch (decode_cfg_valset_state)
			{
				case UBX_DECODE_CFG_VALSET_VERSION:
					cfg_valset_version = bytedata;
					if (cfg_valset_version == 0x00){
						RCLCPP_INFO(this->get_logger(), "UBX-CFG-VALSET Version is 0x00");
					}
					else if(cfg_valset_version ==0x01){
						RCLCPP_INFO(this->get_logger(), "UBX-CFG-VALSET Version is 0x01");
					}else{
						RCLCPP_WARN(this->get_logger(), "Unknown UBX-CFG-VALSET Version: 0x%x", cfg_valset_version);
					}
					decode_cfg_valset_state = UBX_DECODE_CFG_VALSET_LAYERS;
					RCLCPP_INFO(this->get_logger(), "next state is UBX_DECODE_CFG_VALSET_LAYERS, bytedata_remain is %d", bytedata_remain);
				break;

				case UBX_DECODE_CFG_VALSET_LAYERS:
					// is Update configuration in the RAM layer?
					is_ram = bytedata & 0x01;
					// is Update configuration in the BBR layer?
					is_bbr = bytedata & 0x02;
					//is Update configuration in the Flash layer
					is_flash = bytedata & 0x04;
					RCLCPP_INFO(this->get_logger(), "Update configuration layer is: ram-%d, bbr-%d, flash-%d", is_ram, is_bbr, is_flash);

					decode_cfg_valset_state = UBX_DECODE_CFG_VALSET_RES_TRAN;
					RCLCPP_INFO(this->get_logger(), "next state is UBX_DECODE_CFG_VALSET_RES_TRAN, bytedata_remain is %d", bytedata_remain);
				break;

				case UBX_DECODE_CFG_VALSET_RES_TRAN:
					if (cfg_valset_version==1)
					{
						//cfg_valset_version is 1
					}
					decode_cfg_valset_state = UBX_DECODE_CFG_VALSET_RES;
					RCLCPP_INFO(this->get_logger(), "next state is UBX_DECODE_CFG_VALSET_RES, bytedata_remain is %d", bytedata_remain);
				break;

				case UBX_DECODE_CFG_VALSET_RES:
					decode_cfg_valset_state = UBX_DECODE_CFG_VALSET_CFG_ID;
					RCLCPP_INFO(this->get_logger(), "next state is UBX_DECODE_CFG_VALSET_CFG_ID, bytedata_remain is %d", bytedata_remain);
				break;

				case UBX_DECODE_CFG_VALSET_CFG_ID:
					cfg_id |= ((uint32_t)bytedata << (8*cfg_id_index));
					cfg_id_index++;
					if(cfg_id_index>=4)
					{
						//获取完整的cfg_id后，需要获取cfg_value的长度，取决于cfg_id的bit28-bit30位（最低位是bit0）
						//根据cfg_value_size设置cfg_value_byte_size，即决定该cfg_value的长度。
						cfg_value_size = (cfg_id & 0x70000000) >> 28;
						if(cfg_value_size == 1)cfg_value_byte_size=1;
						else if(cfg_value_size == 2)cfg_value_byte_size=1;
						else if(cfg_value_size == 3)cfg_value_byte_size=2;
						else if(cfg_value_size == 4)cfg_value_byte_size=4;
						else if(cfg_value_size == 5)cfg_value_byte_size=8;
						else{}

						//进入下一个状态，准备获取cfg_value
						//cfg_id + cfg_value = cfg_item ，参考ublox的接口文档。
						decode_cfg_valset_state = UBX_DECODE_CFG_VALSET_CFG_VALUE;
						RCLCPP_INFO(this->get_logger(), "next state is UBX_DECODE_CFG_VALSET_CFG_VALUE, cfg_id: 0x%08x, cfg_value_byte_size is %d,bytedate_remain is %d", cfg_id, cfg_value_byte_size, bytedata_remain);
					}else{
						RCLCPP_INFO(this->get_logger(), "current state is UBX_DECODE_CFG_VALSET_CFG_ID, bytedata_remain is: %d, cfg_id_index is %d", bytedata_remain,cfg_id_index);
					}
					
				break;

				case UBX_DECODE_CFG_VALSET_CFG_VALUE:
					//依据 cfg_value_index ，填充_cfg_value，填充长度取决于 cfg_value_byte_size
					_cfg_value.val8byte |= ((uint64_t)bytedata << (8*cfg_value_index));
					cfg_value_index++;

					//如果接收完毕当前的cfg_value，做出对应的响应后，需要重新回到UBX_DECODE_CFG_VALSET_CFG_ID状态，继续获取下一个获取cfg_item的过程
					if(cfg_value_index >= cfg_value_byte_size){
						//code to react to the cfg_item
						react_to_cfg_item(cfg_id, _cfg_value);
						
						//获取cfg_value后拟回到UBX_DECODE_CFG_VALSET_CFG_ID状态继续获取下一组cfg_item
						decode_cfg_valset_state = UBX_DECODE_CFG_VALSET_CFG_ID;
						RCLCPP_INFO(this->get_logger(), "UBX_DECODE_CFG_VALSET_CFG_VALUE is success, next state is UBX_DECODE_CFG_VALSET_CFG_ID");
						//重置状态变量
						decode_cfg_itemInit();
					}else{
						RCLCPP_INFO(this->get_logger(), "current state is UBX_DECODE_CFG_VALSET_CFG_VALUE, bytedata_remain is: %d", bytedata_remain);
					}
					//如果没有剩余待处理字节，那么就结束对当前payload的处理
					if(bytedata_remain==0){
						parse_cfg_valset_payloadInit();
						RCLCPP_INFO(this->get_logger(), "Current cfg_valset_payload has been parsed done!");
					}
				break;

				default:
					break;
			}
			
		}

		void parse_cfg_valset_payloadInit(void){
			decode_cfg_valset_state = UBX_DECODE_CFG_VALSET_VERSION;
			cfg_valset_version = 0x00;
		}

		void decode_cfg_itemInit(){
			cfg_id = 0;
			cfg_id_index = 0;
			cfg_value_byte_size = 0;
			cfg_value_index = 0;
			_cfg_value.val8byte = 0;
		}

		void react_to_cfg_item(uint32_t cfg_id, cfg_value_t _cfg_value)
		{
			
			switch (cfg_id)
			{
			case UBX_CFG_KEY_CFG_UART1_BAUDRATE:
				RCLCPP_INFO(this->get_logger(), "Flight Controller want UART-BAUDRATE to be %d", _cfg_value.val4byte);
				break;

			case UBX_CFG_KEY_CFG_UART1_STOPBITS:
				/* code */
				break;
			
			case UBX_CFG_KEY_CFG_UART1_DATABITS:
				/* code */
				break;
			
			case UBX_CFG_KEY_CFG_UART1_PARITY:
				/* code */
				break;
			
			case UBX_CFG_KEY_CFG_UART1INPROT_UBX:
				/* code */
				break;

			case UBX_CFG_KEY_CFG_UART1INPROT_RTCM3X:
				/* code */
				break;

			case UBX_CFG_KEY_CFG_UART1INPROT_NMEA:
				break;

			case UBX_CFG_KEY_CFG_UART1OUTPROT_UBX:
				break;

			case UBX_CFG_KEY_CFG_UART1OUTPROT_NMEA:
				break;

			case UBX_CFG_KEY_CFG_USBINPROT_UBX:
				break;

			case UBX_CFG_KEY_CFG_USBINPROT_RTCM3X:
				break;

			case UBX_CFG_KEY_CFG_USBINPROT_NMEA:
				break;

			case UBX_CFG_KEY_CFG_USBOUTPROT_UBX:
				break;

			case UBX_CFG_KEY_CFG_USBOUTPROT_NMEA:
				break;

			default:
				RCLCPP_INFO(this->get_logger(), "obtain complete cfg_item. cfg_id: 0x%08x, cfg_value: %016lx", cfg_id, _cfg_value.val8byte);
				break;
			}
		}

	private:
		rclcpp::Subscription<adahil_interface::msg::GPSData>::SharedPtr gps_sub;

		typedef enum ubx_decode_cfg_valset_state
		{
			UBX_DECODE_CFG_VALSET_VERSION = 0,
			UBX_DECODE_CFG_VALSET_LAYERS,
			UBX_DECODE_CFG_VALSET_RES_TRAN,
			UBX_DECODE_CFG_VALSET_RES,
			UBX_DECODE_CFG_VALSET_CFG_ID,
			UBX_DECODE_CFG_VALSET_CFG_VALUE,
		} ubx_decode_cfg_valset_state_t;

		typedef enum ubx_decode_state
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
		} ubx_decode_state_t; 

		ubx_decode_state_t _decode_state;

		uint8_t _rx_ck_a{0};
		uint8_t _rx_ck_b{0};
		uint16_t _rx_payload_length{0};
		uint16_t _rx_payload_index{0};
		uint16_t _rx_msg{};
		uint8_t _payload_buf[256];//payload buffer

		uint8_t _tx_ck_a{0};
		uint8_t _tx_ck_b{0};

		ubx_decode_cfg_valset_state_t decode_cfg_valset_state;
		uint8_t cfg_valset_version{0};
		uint8_t cfg_id_index{0};
		uint32_t cfg_id{0};
		uint8_t cfg_value_byte_size{0};
		uint8_t cfg_value_index{0};

		cfg_value_t _cfg_value{0};
		// uint8_t cfg_value_1byte{0};
		// uint16_t cfg_value_2byte{0};
		// uint32_t cfg_value_4byte{0};
		// uint64_t cfg_value_8byte{0};
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
