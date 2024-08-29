#ifndef ORDER_PUBLISHER_H
#define ORDER_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "order_publisher/msg/order.hpp"

class OrderPublisher : public rclcpp::Node
{
public:

    OrderPublisher();

private:

    void publish_order();

    int get_order_id();

    rclcpp::Publisher<order_publisher::msg::Order>::SharedPtr _order_publisher;
    rclcpp::TimerBase::SharedPtr _order_timer;
    int _order_count;

};

#endif