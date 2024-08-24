#ifndef ORDER_PUBLISHER_H
#define ORDER_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "order_publisher/msg/order.hpp"

class OrderPublisher
{
public:

private:
    rclcpp::Publisher<order_publisher::msg::Order>::SharedPtr _order_publisher;

};

#endif