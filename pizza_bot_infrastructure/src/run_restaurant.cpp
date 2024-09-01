#include "restaurant.h"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Restaurant>());
	rclcpp::shutdown();
	return 0;
}