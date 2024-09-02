#ifndef CUSTOMER_H
#define CUSTOMER_H

#include "rclcpp/rclcpp.hpp"
#include "pizza_bot_interfaces/msg/order.hpp"
#include "pizza_bot_interfaces/srv/deliver_pizza.hpp"

class Customer : public rclcpp::Node
{
public:

    Customer();

private:

    void received_order_callback(pizza_bot_interfaces::msg::Order::SharedPtr received_order);

    void pizza_delivery(const std::shared_ptr<pizza_bot_interfaces::srv::DeliverPizza::Request> request,
        std::shared_ptr<pizza_bot_interfaces::srv::DeliverPizza::Response> response);

    rclcpp::Subscription<pizza_bot_interfaces::msg::Order>::SharedPtr _received_order_subscriber;
    rclcpp::Service<pizza_bot_interfaces::srv::DeliverPizza>::SharedPtr _deliver_pizza_service;
};

#endif