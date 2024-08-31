#include "pizza_bot_controller.h"

PizzaBotController::PizzaBotController()
    : Node("pizza_bot_controller_node")
{
    _order_subscriber = create_subscription<pizza_bot_interfaces::msg::Order>("orders", 
        10, 
        std::bind(&PizzaBotController::order_callback, 
            this, 
            std::placeholders::_1));

    _received_order_publisher = create_publisher<pizza_bot_interfaces::msg::Order>("received_orders", 10);
}

void PizzaBotController::order_callback(pizza_bot_interfaces::msg::Order::SharedPtr order)
{
    _received_order_publisher->publish(*order);
}