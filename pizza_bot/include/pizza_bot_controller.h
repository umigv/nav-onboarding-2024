#ifndef PIZZA_BOT_CONTROLLER_H
#define PIZZA_BOT_CONTROLLER_H

#include "rclcpp/rclcpp.hpp"
#include "pizza_bot_interfaces/msg/order.hpp"
#include "pizza_bot_interfaces/srv/navigate_to_coord.hpp"

class PizzaBotController : public rclcpp::Node
{
public:

    PizzaBotController();

private:

    void order_callback(pizza_bot_interfaces::msg::Order::SharedPtr order);

    // Calls the navigate_to_coord service with the given goal asynchronously
    // The given response_callback will be called with the response once the service finishes
    void call_navigate_service(const pizza_bot_interfaces::msg::Coord &goal,
        std::function<void(rclcpp::Client<pizza_bot_interfaces::srv::NavigateToCoord>::SharedFuture)> response_callback);

    void pizza_place_navigation_callback(rclcpp::Client<pizza_bot_interfaces::srv::NavigateToCoord>::SharedFuture future,
        const pizza_bot_interfaces::msg::Coord &goal);

    rclcpp::Subscription<pizza_bot_interfaces::msg::Order>::SharedPtr _order_subscriber;
    rclcpp::Publisher<pizza_bot_interfaces::msg::Order>::SharedPtr _received_order_publisher;
    rclcpp::Client<pizza_bot_interfaces::srv::NavigateToCoord>::SharedPtr _navigate_client;
};

#endif