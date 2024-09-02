#include "navigator.h"

#include <chrono>

using namespace std::chrono_literals;
using namespace std::placeholders;
using NavigateToCoord = pizza_bot_interfaces::srv::NavigateToCoord;

Navigator::Navigator()
    : Node("navigator_node")
{
    declare_parameter("navigation_delay_seconds", 2);

    _navigate_service = create_service<NavigateToCoord>("navigate_to_coord", 
        std::bind(&Navigator::navigate_to_coord, 
            this, 
            _1, 
            _2));
}

void Navigator::navigate_to_coord(const std::shared_ptr<NavigateToCoord::Request> request,
        std::shared_ptr<NavigateToCoord::Response> response)
{
    RCLCPP_INFO(get_logger(),
        "Received request to navigate to (%ld, %ld)",
        request->goal.x,
        request->goal.y);

    int navigation_delay = get_parameter("navigation_delay_seconds").as_int();
    rclcpp::sleep_for(std::chrono::seconds(navigation_delay));

    response->success = true;
    RCLCPP_INFO(get_logger(),
        "Navigation successful");
}