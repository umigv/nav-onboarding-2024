#include "navigator.h"

#include <chrono>

using namespace std::chrono_literals;

Navigator::Navigator()
    : Node("navigator_node")
{
    _navigate_service = create_service<pizza_bot_interfaces::srv::NavigateToCoord>("navigate_to_coord", 
        std::bind(&Navigator::navigate_to_coord, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2));
}

void Navigator::navigate_to_coord(const std::shared_ptr<pizza_bot_interfaces::srv::NavigateToCoord::Request> request,
        std::shared_ptr<pizza_bot_interfaces::srv::NavigateToCoord::Response> response)
{
    RCLCPP_INFO(get_logger(),
        "Received request to navigate to (%ld, %ld)",
        request->goal.x,
        request->goal.y);
    response->success = true;
    RCLCPP_INFO(get_logger(),
        "Navigation successful");
}