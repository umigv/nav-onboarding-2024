#include "pizza_bot_controller.h"

#include <chrono>
#include <memory>
#include <functional>

using namespace std::chrono_literals;
using namespace std::placeholders;
using Order = pizza_bot_interfaces::msg::Order;
using Coord = pizza_bot_interfaces::msg::Coord;
using NavigateToCoord = pizza_bot_interfaces::srv::NavigateToCoord;
using NavigateToCoordSharedFuture = rclcpp::Client<NavigateToCoord>::SharedFuture;
using MakePizza = pizza_bot_interfaces::action::MakePizza;
using MakePizzaGoalHandle = rclcpp_action::ClientGoalHandle<MakePizza>;
using DeliverPizza = pizza_bot_interfaces::srv::DeliverPizza;
using DeliverPizzaSharedFuture = rclcpp::Client<DeliverPizza>::SharedFuture;

PizzaBotController::PizzaBotController()
    : Node("pizza_bot_controller_node")
{
    _order_subscriber = create_subscription<Order>("orders", 
        10, 
        std::bind(&PizzaBotController::order_callback, 
            this, 
            _1));

    _received_order_publisher = create_publisher<Order>("received_orders", 10);
    _navigate_client = create_client<NavigateToCoord>("navigate_to_coord");
    _make_pizza_client = rclcpp_action::create_client<MakePizza>(this, "make_pizza");
    _deliver_pizza_client = create_client<DeliverPizza>("deliver_pizza");
}

void PizzaBotController::order_callback(Order::SharedPtr order)
{
    RCLCPP_INFO(get_logger(),
        "---------- Publishing received order ----------");
    _received_order_publisher->publish(*order);
    call_navigate_service(order->pizza_place_coord,
        std::bind(&PizzaBotController::pizza_place_navigation_callback,
            this,
            _1,
            order));
}

void PizzaBotController::call_navigate_service(const Coord &goal,
    std::function<void(NavigateToCoordSharedFuture)> response_callback)
{
    while (!_navigate_client->wait_for_service(1s)) 
	{
	    RCLCPP_INFO(get_logger(), 
            "navigate_to_coord service not available, waiting again...");
	}

    auto goal_request = std::make_shared<NavigateToCoord::Request>();
    goal_request->goal = goal;

    _navigate_client->async_send_request(goal_request, response_callback);
}

void PizzaBotController::pizza_place_navigation_callback(NavigateToCoordSharedFuture future,
    const Order::SharedPtr order)
{
    bool navigation_succeeded = future.get()->success;
    Coord pizza_place_coord = order->pizza_place_coord;
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

void PizzaBotController::send_pizza_order(const Order::SharedPtr order)
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
    const Order::SharedPtr order)
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

    call_navigate_service(order->customer_coord,
        std::bind(&PizzaBotController::customer_navigation_callback,
            this,
            _1,
            order));
}

void PizzaBotController::customer_navigation_callback(rclcpp::Client<NavigateToCoord>::SharedFuture future,
    const Order::SharedPtr order)
{
    bool navigation_succeeded = future.get()->success;
    Coord customer_coord = order->customer_coord;
    if (!navigation_succeeded)
    {
        RCLCPP_INFO(get_logger(),
            "Failed to navigate to customer's house at (%ld, %ld)",
            customer_coord.x,
            customer_coord.y);
        return;
    }

    RCLCPP_INFO(get_logger(),
        "Successfuly navigated to customer's house at (%ld, %ld)",
        customer_coord.x,
        customer_coord.y);

    deliver_pizza(order->pizza_type);
}

void PizzaBotController::deliver_pizza(const std::string &pizza_type)
{
    while (!_deliver_pizza_client->wait_for_service(1s)) 
	{
	    RCLCPP_INFO(get_logger(), 
            "deliver_pizza service not available, waiting again...");
	}

    auto delivery_request = std::make_shared<DeliverPizza::Request>();
    delivery_request->pizza_type = pizza_type;

    _deliver_pizza_client->async_send_request(delivery_request, 
        std::bind(&PizzaBotController::pizza_delivery_callback,
            this,
            _1));
}

void PizzaBotController::pizza_delivery_callback(DeliverPizzaSharedFuture future)
{
    bool delivery_succeeded = future.get()->success;
    if (!delivery_succeeded)
    {
        RCLCPP_INFO(get_logger(),
            "Failed to complete delivery");
        return;
    }

    RCLCPP_INFO(get_logger(),
        "Successfuly completed delivery");
}