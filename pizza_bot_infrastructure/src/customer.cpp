#include "customer.h"

Customer::Customer()
    : Node("customer_node")
{
    _received_order_subscriber = create_subscription<pizza_bot_interfaces::msg::Order>("received_orders", 
        10, 
        std::bind(&Customer::received_order_callback, 
            this, 
            std::placeholders::_1));
}

void Customer::received_order_callback(pizza_bot_interfaces::msg::Order::SharedPtr received_order)
{
    RCLCPP_INFO(get_logger(),
        "Great! I can't wait for my %s pizza from %s",
        received_order->pizza_type.c_str(),
        received_order->pizza_place.c_str());
}