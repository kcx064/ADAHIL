#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
	/* init rclcpp */
	rclcpp::init(argc, argv);
	/* create a node named node_01 */
	auto node = std::make_shared<rclcpp::Node>("node_01");
	// print introduction
	RCLCPP_INFO(node->get_logger(), "node_01节点已经启动");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}