#ifndef PIZZA_BOT_CONTROLLER_H
#define PIZZA_BOT_CONTROLLER_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pizza_bot_interfaces/msg/order.hpp"
#include "pizza_bot_interfaces/srv/navigate_to_coord.hpp"
#include "pizza_bot_interfaces/action/make_pizza.hpp"
#include "pizza_bot_interfaces/srv/deliver_pizza.hpp"

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
        const pizza_bot_interfaces::msg::Order::SharedPtr order);

    void send_pizza_order(const pizza_bot_interfaces::msg::Order::SharedPtr order);

    void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<pizza_bot_interfaces::action::MakePizza>> goal_handle);

    void feedback_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<pizza_bot_interfaces::action::MakePizza>> goal_handle,
        const std::shared_ptr<const pizza_bot_interfaces::action::MakePizza::Feedback> feedback);

    void result_callback(const rclcpp_action::ClientGoalHandle<pizza_bot_interfaces::action::MakePizza>::WrappedResult &result,
        const pizza_bot_interfaces::msg::Order::SharedPtr order);

    void customer_navigation_callback(rclcpp::Client<pizza_bot_interfaces::srv::NavigateToCoord>::SharedFuture future,
        const pizza_bot_interfaces::msg::Order::SharedPtr order);

    void deliver_pizza(const std::string &pizza_type);

    void pizza_delivery_callback(rclcpp::Client<pizza_bot_interfaces::srv::DeliverPizza>::SharedFuture future);

    rclcpp::Subscription<pizza_bot_interfaces::msg::Order>::SharedPtr _order_subscriber;
    rclcpp::Publisher<pizza_bot_interfaces::msg::Order>::SharedPtr _received_order_publisher;
    rclcpp::Client<pizza_bot_interfaces::srv::NavigateToCoord>::SharedPtr _navigate_client;
    rclcpp_action::Client<pizza_bot_interfaces::action::MakePizza>::SharedPtr _make_pizza_client;
    rclcpp::Client<pizza_bot_interfaces::srv::DeliverPizza>::SharedPtr _deliver_pizza_client;
};

#endif