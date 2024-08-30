#ifndef ORDER_PUBLISHER_H
#define ORDER_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "pizza_bot_infrastructure/msg/order.hpp"
#include "nlohmann/json.hpp"

class OrderPublisher : public rclcpp::Node
{
public:

    OrderPublisher();

private:

    void publish_order();

    rclcpp::Publisher<pizza_bot_infrastructure::msg::Order>::SharedPtr _order_publisher;
    rclcpp::TimerBase::SharedPtr _order_timer;

    nlohmann::json _orders;
    size_t _order_count;
};

#endif