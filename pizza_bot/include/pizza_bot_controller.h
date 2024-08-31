#ifndef PIZZA_BOT_CONTROLLER_H
#define PIZZA_BOT_CONTROLLER_H

#include "rclcpp/rclcpp.hpp"
#include "pizza_bot_interfaces/msg/order.hpp"

class PizzaBotController : public rclcpp::Node
{
public:

    PizzaBotController();

private:

    void order_callback(pizza_bot_interfaces::msg::Order::SharedPtr order);

    rclcpp::Subscription<pizza_bot_interfaces::msg::Order>::SharedPtr _order_subscriber;
    rclcpp::Publisher<pizza_bot_interfaces::msg::Order>::SharedPtr _received_order_publisher;
};

#endif