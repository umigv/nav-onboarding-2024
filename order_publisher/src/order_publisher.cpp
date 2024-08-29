#include "order_publisher.h"

#include <chrono>
#include <memory>
#include <iostream>

using namespace std::chrono_literals;

OrderPublisher::OrderPublisher()
    : Node("order_publisher_node"),
    _order_count(0)
{
    _order_publisher = create_publisher<order_publisher::msg::Order>("orders", 10);
    _order_timer = create_wall_timer(1s, 
    std::bind(&OrderPublisher::publish_order, 
    this));
}

void OrderPublisher::publish_order()
{
    std::cout << "Hello from publish_order\n";

    order_publisher::msg::Order order_message;
    order_message.order_id = get_order_id();
}

int OrderPublisher::get_order_id()
{
    int order_id = _order_count;
    _order_count++;
    return order_id;
}