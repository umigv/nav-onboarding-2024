#ifndef RESTAURANT_H
#define RESTAURANT_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pizza_bot_interfaces/action/make_pizza.hpp"

class Restaurant : public rclcpp::Node
{
public: 

    Restaurant();

private: 

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const pizza_bot_interfaces::action::MakePizza::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<pizza_bot_interfaces::action::MakePizza>> goal_handle);

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<pizza_bot_interfaces::action::MakePizza>> goal_handle);

    void make_pizza(const std::shared_ptr<rclcpp_action::ServerGoalHandle<pizza_bot_interfaces::action::MakePizza>> goal_handle);

    rclcpp_action::Server<pizza_bot_interfaces::action::MakePizza>::SharedPtr _pizza_action_server;
};

#endif