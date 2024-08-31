#include "order_publisher.h"

#include <chrono>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>

using json = nlohmann::json;
using namespace std::chrono_literals;

OrderPublisher::OrderPublisher()
    : Node("order_publisher_node"),
    _order_count(0)
{
    _order_publisher = create_publisher<pizza_bot_interfaces::msg::Order>("orders", 10);
    _order_timer = create_wall_timer(1s, 
        std::bind(&OrderPublisher::publish_order, 
        this));

    declare_parameter("orders_path", "orders.json");
    declare_parameter("repeat_orders", true);

    // Open and parse orders json file only once
    std::ifstream orders_file(get_parameter("orders_path").as_string());
    json orders_json = json::parse(orders_file);
    _orders = orders_json["orders"];
}

void OrderPublisher::publish_order()
{
    if (_order_count >= _orders.size())
    {
        bool repeat_orders = get_parameter("repeat_orders").as_bool();
        if (repeat_orders)
        {
            _order_count = 0;
        }
        else
        {
            RCLCPP_INFO(get_logger(),
                "All orders published\n");
            _order_timer->cancel();
            return;
        }
    }
    
    json order = _orders[_order_count];
    
    pizza_bot_interfaces::msg::Order order_message;
    order_message.order_id = _order_count;
    order_message.pizza_place = order["pizza_place"].template get<std::string>();
    order_message.pizza_type = order["pizza_type"].template get<std::string>();

    pizza_bot_interfaces::msg::Coord pizza_place_coord, customer_coord;
    pizza_place_coord.x = order["pizza_place_coord"][0];
    pizza_place_coord.y = order["pizza_place_coord"][1];
    customer_coord.x = order["customer_coord"][0];
    customer_coord.y = order["customer_coord"][1];

    order_message.pizza_place_coord = pizza_place_coord;
    order_message.customer_coord = customer_coord;

    _order_publisher->publish(order_message);
    ++_order_count;
}
