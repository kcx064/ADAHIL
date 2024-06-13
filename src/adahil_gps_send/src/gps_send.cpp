#include <cstdio>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "adahil_interface/msg/gps_data.hpp"
#include "rclcpp/rclcpp.hpp"

class GpsSend : public rclcpp::Node
{
	public:
		int fd;
		GpsSend(std::string name) : Node(name)
		{
			RCLCPP_INFO(this->get_logger(), "Node is running: %s.", name.c_str());

			fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
			if (fd == -1){
				RCLCPP_ERROR(this->get_logger(), "UART open Error, fd: %d.", fd);
			}else{
				RCLCPP_INFO(this->get_logger(), "UART open Success, fd: %d.", fd);
			}

			gps_sub = this->create_subscription<adahil_interface::msg::GPSData>("gps_data", 5, std::bind(&GpsSend::gps_callback, this, std::placeholders::_1));
			
		}

		void gps_callback(const adahil_interface::msg::GPSData::SharedPtr msg)
		{
			RCLCPP_INFO(this->get_logger(), "GPS Data: %f, %f.", msg->lon, msg->lat);
		}

	private:
		rclcpp::Subscription<adahil_interface::msg::GPSData>::SharedPtr gps_sub;
		
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<GpsSend>("gps_send");
	rclcpp::spin(node);
	rclcpp::shutdown();
	close(node->fd);
  	return 0;
}
