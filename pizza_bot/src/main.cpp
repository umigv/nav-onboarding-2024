#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "pizza_bot_controller.h"

int main(int argc, char * argv[])
{   
    std::cout << "Beep Boop im a pizza bot\n";
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PizzaBotController>());
    rclcpp::shutdown();
    return 0;
}
