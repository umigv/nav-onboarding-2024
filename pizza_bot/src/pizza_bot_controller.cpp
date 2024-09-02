#include "pizza_bot_controller.h"

#include <chrono>
#include <memory>
#include <functional>

using namespace std::chrono_literals;
using NavigateToCoordSharedFuture = rclcpp::Client<pizza_bot_interfaces::srv::NavigateToCoord>::SharedFuture;

PizzaBotController::PizzaBotController()
    : Node("pizza_bot_controller_node")
{
    _order_subscriber = create_subscription<pizza_bot_interfaces::msg::Order>("orders", 
        10, 
        std::bind(&PizzaBotController::order_callback, 
            this, 
            std::placeholders::_1));

    _received_order_publisher = create_publisher<pizza_bot_interfaces::msg::Order>("received_orders", 10);
    _navigate_client = create_client<pizza_bot_interfaces::srv::NavigateToCoord>("navigate_to_coord");
}

void PizzaBotController::order_callback(pizza_bot_interfaces::msg::Order::SharedPtr order)
{
    RCLCPP_INFO(get_logger(),
        "Publishing received order");
    _received_order_publisher->publish(*order);
    call_navigate_service(order->pizza_place_coord,
        std::bind(&PizzaBotController::pizza_place_navigation_callback,
            this,
            std::placeholders::_1,
            order->pizza_place_coord));
}

void PizzaBotController::call_navigate_service(const pizza_bot_interfaces::msg::Coord &goal,
    std::function<void(NavigateToCoordSharedFuture)> response_callback)
{
    while (!_navigate_client->wait_for_service(1s)) 
	{
	    RCLCPP_INFO(get_logger(), 
            "navigate_to_coord service not available, waiting again...");
	}

    auto goal_request = std::make_shared<pizza_bot_interfaces::srv::NavigateToCoord::Request>();
    goal_request->goal = goal;

    auto result = _navigate_client->async_send_request(goal_request, response_callback);
}

void PizzaBotController::pizza_place_navigation_callback(NavigateToCoordSharedFuture future,
    const pizza_bot_interfaces::msg::Coord &goal)
{
    bool navigation_succeeded = future.get()->success;
    if (navigation_succeeded)
    {
        RCLCPP_INFO(get_logger(),
            "Successfuly navigated to (%ld, %ld)",
            goal.x,
            goal.y);
    }
    else
    {
        RCLCPP_INFO(get_logger(),
            "Failed to navigate to (%ld, %ld)",
            goal.x,
            goal.y);
    }
}