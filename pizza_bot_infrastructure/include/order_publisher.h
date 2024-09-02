#ifndef ORDER_PUBLISHER_H
#define ORDER_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "pizza_bot_interfaces/msg/order.hpp"
#include "nlohmann/json.hpp"

class OrderPublisher : public rclcpp::Node
{
public:

    OrderPublisher();

private:

    void publish_order();

    rclcpp::Publisher<pizza_bot_interfaces::msg::Order>::SharedPtr _order_publisher;
    rclcpp::TimerBase::SharedPtr _order_timer;

    nlohmann::json _orders;
    size_t _order_count;
    bool _first_order;
};

#endif