#include "customer.h"

using namespace std::placeholders;
using DeliverPizza = pizza_bot_interfaces::srv::DeliverPizza;
using Order = pizza_bot_interfaces::msg::Order;

Customer::Customer()
    : Node("customer_node")
{
    _received_order_subscriber = create_subscription<Order>("received_orders", 
        10, 
        std::bind(&Customer::received_order_callback, 
            this, 
            _1));
    _deliver_pizza_service = create_service<DeliverPizza>("deliver_pizza", 
        std::bind(&Customer::pizza_delivery, 
            this, 
            _1, 
            _2));
}

void Customer::received_order_callback(Order::SharedPtr received_order)
{
    RCLCPP_INFO(get_logger(),
        "Great! I can't wait for my %s pizza from %s",
        received_order->pizza_type.c_str(),
        received_order->pizza_place.c_str());
}

void Customer::pizza_delivery(const std::shared_ptr<DeliverPizza::Request> request,
    std::shared_ptr<DeliverPizza::Response> response)
{
    response->success = true;
    RCLCPP_INFO(get_logger(),
        "Thank you for the %s pizza!",
        request->pizza_type.c_str());
}