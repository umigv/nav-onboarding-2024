#include "pizza_bot_controller.h"

#include <chrono>
#include <memory>
#include <functional>

using namespace std::chrono_literals;
using namespace std::placeholders;
using NavigateToCoordSharedFuture = rclcpp::Client<pizza_bot_interfaces::srv::NavigateToCoord>::SharedFuture;
using MakePizza = pizza_bot_interfaces::action::MakePizza;
using MakePizzaGoalHandle = rclcpp_action::ClientGoalHandle<MakePizza>;

PizzaBotController::PizzaBotController()
    : Node("pizza_bot_controller_node")
{
    _order_subscriber = create_subscription<pizza_bot_interfaces::msg::Order>("orders", 
        10, 
        std::bind(&PizzaBotController::order_callback, 
            this, 
            _1));

    _received_order_publisher = create_publisher<pizza_bot_interfaces::msg::Order>("received_orders", 10);
    _navigate_client = create_client<pizza_bot_interfaces::srv::NavigateToCoord>("navigate_to_coord");
    _make_pizza_client = rclcpp_action::create_client<MakePizza>(this, "make_pizza");
}

void PizzaBotController::order_callback(pizza_bot_interfaces::msg::Order::SharedPtr order)
{
    RCLCPP_INFO(get_logger(),
        "Publishing received order");
    _received_order_publisher->publish(*order);
    call_navigate_service(order->pizza_place_coord,
        std::bind(&PizzaBotController::pizza_place_navigation_callback,
            this,
            _1,
            order));
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
    const pizza_bot_interfaces::msg::Order::SharedPtr order)
{
    bool navigation_succeeded = future.get()->success;
    pizza_bot_interfaces::msg::Coord pizza_place_coord = order->pizza_place_coord;
    std::string pizza_place_name = order->pizza_place;
    if (!navigation_succeeded)
    {
        RCLCPP_INFO(get_logger(),
            "Failed to navigate to %s at (%ld, %ld)",
            pizza_place_name.c_str(),
            pizza_place_coord.x,
            pizza_place_coord.y);
        return;
    }

    RCLCPP_INFO(get_logger(),
        "Successfuly navigated to %s at (%ld, %ld)",
        pizza_place_name.c_str(),
        pizza_place_coord.x,
        pizza_place_coord.y);

    send_pizza_order(order);
}

void PizzaBotController::send_pizza_order(const pizza_bot_interfaces::msg::Order::SharedPtr order)
{
    while (!_make_pizza_client->wait_for_action_server(1s))
    {
        RCLCPP_INFO(get_logger(), 
            "make_pizza action not available, waiting again...");
    }

    MakePizza::Goal pizza_order;
    pizza_order.pizza_type = order->pizza_type;

    rclcpp_action::Client<MakePizza>::SendGoalOptions options;
    options.goal_response_callback = std::bind(&PizzaBotController::goal_response_callback,
        this, 
        _1);
    options.feedback_callback = std::bind(&PizzaBotController::feedback_callback,
        this, 
        _1,
        _2);
    options.result_callback = std::bind(&PizzaBotController::result_callback,
        this, 
        _1,
        order);

    _make_pizza_client->async_send_goal(pizza_order, options);
}

void PizzaBotController::goal_response_callback(std::shared_ptr<MakePizzaGoalHandle> goal_handle)
{
    if (!goal_handle) 
    {
        RCLCPP_ERROR(get_logger(), 
            "Order was rejected by restaurant");
    } 
    else 
    {
        RCLCPP_INFO(get_logger(), 
            "Order accepted by restaurant, waiting for pizza");
    }
}

void PizzaBotController::feedback_callback(std::shared_ptr<MakePizzaGoalHandle> goal_handle,
    const std::shared_ptr<const MakePizza::Feedback> feedback)
{
    (void)goal_handle; // Prevent unused parameter warning
    RCLCPP_INFO(get_logger(),
        "Feedback from restaurant: %s",
        feedback->pizza_status.c_str());
}

void PizzaBotController::result_callback(const MakePizzaGoalHandle::WrappedResult &result,
    const pizza_bot_interfaces::msg::Order::SharedPtr order)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;

    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(),
            "Order was aborted");
        return;

    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(),
            "Order was canceled");
        return;

    default:
        RCLCPP_ERROR(get_logger(),
            "Unknown result code");
        break;
    }

    std::string pizza_type = order->pizza_type;
    std::string pizza_place = order->pizza_place;

    if (!result.result->pizza_finished)
    {
        RCLCPP_INFO(get_logger(),
            "%s failed to complete the order for a %s pizza",
            pizza_place.c_str(),
            pizza_type.c_str());
        return;
    }

    RCLCPP_INFO(get_logger(),
        "%s completed the order for a %s pizza",
        pizza_place.c_str(),
        pizza_type.c_str());

    
}