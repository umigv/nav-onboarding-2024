#ifndef PIZZA_BOT_CONTROLLER_H
#define PIZZA_BOT_CONTROLLER_H

#include "rclcpp/rclcpp.hpp"

#include "pizza_bot_interfaces/msg/order.hpp"
#include "pizza_bot_interfaces/srv/navigate_to_coord.hpp"
#include "pizza_bot_interfaces/srv/deliver_pizza.hpp"
class PizzaBotController  : public rclcpp::Node
{
public:
    PizzaBotController ();

private:
    void order_callback(const pizza_bot_interfaces::msg::Order::SharedPtr msg);
    void navigate_to_coord(pizza_bot_interfaces::msg::Order::SharedPtr ord);
    void deliver_pizza(pizza_bot_interfaces::msg::Order::SharedPtr);
    void PizzaBotController::pizza_navigation_callback(rclcpp::Client<pizza_bot_interfaces::srv::NavigateToCoord>::SharedFuture future,
        pizza_bot_interfaces::msg::Order::SharedPtr ord)    
    void delivery_bot_callback(FILL OUT );
    rclcpp::Subscription<pizza_bot_interfaces::msg::Order>::SharedPtr subscription_;
    rclcpp::Publisher<pizza_bot_interfaces::msg::Order>::SharedPtr publisher_;
    rclcpp::Client<FILL OUT>::SharedPtr client_;
    rclcpp::Client<FILL OUT>::SharedPtr delivery_client_;

};

#endif  // ORDER_SUBSCRIBER_HPP_
