#include <memory>
#include  <chrono>
#include "rclcpp/rclcpp.hpp"
#include "pizza_bot_controller.h"
#include <string>
#include <iostream>
#include <functional>



PizzaBotController::PizzaBotController ()
    : Node("pizza_bot_controller")
{
    subscription_ = this->create_subscription<pizza_bot_interfaces::msg::Order>(
        "/orders", 10, std::bind(&PizzaBotController ::order_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<pizza_bot_interfaces::msg::Order>(
        "received_orders", 10);
    client_ = this->create_client<pizza_bot_interfaces::srv::NavigateToCoord>("navigate_to_coord");
    delivery_client_ = this->create_client<pizza_bot_interfaces::srv::DeliverPizza>("deliver_pizza");
}

void PizzaBotController::order_callback(const pizza_bot_interfaces::msg::Order::SharedPtr msg)
{
        RCLCPP_INFO(this->get_logger(), "Receiving");
        RCLCPP_INFO(this->get_logger(), "Order ID: %ld", msg->order_id);
        RCLCPP_INFO(this->get_logger(), "Pizza Place: %s", msg->pizza_place.c_str());
        RCLCPP_INFO(this->get_logger(), "Pizza Type: %s", msg->pizza_type.c_str());
        RCLCPP_INFO(this->get_logger(), "Pizza Place Coordinates: (%ld, %ld)", msg->pizza_place_coord.x, msg->pizza_place_coord.y);
        RCLCPP_INFO(this->get_logger(), "Customer Coordinates: (%ld, %ld)", msg->customer_coord.x, msg->customer_coord.y);
        
        publisher_->publish(*msg);
    // Call the NavigateToCoord service with the order's x and y coordinates
        navigate_to_coord(msg);
    
}

void PizzaBotController::navigate_to_coord(pizza_bot_interfaces::msg::Order::SharedPtr ord)
{
    RCLCPP_INFO(this->get_logger(), "Navigating");

    int x = ord->pizza_place_coord.x;
    int y = ord->pizza_place_coord.y;
    auto request = std::make_shared<pizza_bot_interfaces::srv::NavigateToCoord::Request>();
    pizza_bot_interfaces::msg::Coord gps;
    gps.x = x;
    gps.y = y;
    request->goal = gps;
    std::chrono::seconds duration(2);
    while (!client_->wait_for_service(duration)) 
	{
	    RCLCPP_INFO(get_logger(), 
            "navigate_to_coord service not available, waiting...");
	}

    const pizza_bot_interfaces::msg::Order::SharedPtr order = ord;
  
     client_->async_send_request(
        request,
        [this, order](rclcpp::Client<pizza_bot_interfaces::srv::NavigateToCoord>::SharedFuture future) {
            this->pizza_navigation_callback(future, order);
        }
    );
  
}

void PizzaBotController::deliver_pizza(pizza_bot_interfaces::msg::Order::SharedPtr ord)
{

    RCLCPP_INFO(this->get_logger(), "Delivering");

    auto request = std::make_shared<pizza_bot_interfaces::srv::DeliverPizza::Request>();
    std::string pizza;
    pizza = (std::string)ord->pizza_type;
    request->pizza_type = pizza;
    std::chrono::seconds duration(1);
    while (!client_->wait_for_service(duration)) 
	{
	    RCLCPP_INFO(get_logger(), 
            "delivery_pizza service not available, waiting...");
	}

  

    delivery_client_->async_send_request(
        request
      );
            
     
}

void PizzaBotController::pizza_navigation_callback(rclcpp::Client<pizza_bot_interfaces::srv::NavigateToCoord>::SharedFuture future,
pizza_bot_interfaces::msg::Order::SharedPtr ord)
{
    bool navigation_succeeded = future.get()->success;
    if (!navigation_succeeded)
    {
        RCLCPP_INFO(get_logger(),
            "Failed to complete navigation");
        return;
    }

    RCLCPP_INFO(get_logger(),
        "Successfuly completed navigation");
    deliver_pizza(ord);
}
