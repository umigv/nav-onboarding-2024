#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "rclcpp/rclcpp.hpp"
#include "pizza_bot_interfaces/srv/navigate_to_coord.hpp"

class Navigator : public rclcpp::Node
{
public: 

    Navigator();

private:

    void navigate_to_coord(const std::shared_ptr<pizza_bot_interfaces::srv::NavigateToCoord::Request> request,
        std::shared_ptr<pizza_bot_interfaces::srv::NavigateToCoord::Response> response);

    rclcpp::Service<pizza_bot_interfaces::srv::NavigateToCoord>::SharedPtr _navigate_service;
};


#endif