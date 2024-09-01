#include "navigator.h"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Navigator>());
	rclcpp::shutdown();
	return 0;
}