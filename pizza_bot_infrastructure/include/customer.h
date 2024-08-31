#ifndef CUSTOMER_H
#define CUSTOMER_H

#include "rclcpp/rclcpp.hpp"
#include "pizza_bot_interfaces/msg/order.hpp"

class Customer : public rclcpp::Node
{
public:

    Customer();

private:

    void received_order_callback(pizza_bot_interfaces::msg::Order::SharedPtr received_order);

    rclcpp::Subscription<pizza_bot_interfaces::msg::Order>::SharedPtr _received_order_subscriber;
};

#endif