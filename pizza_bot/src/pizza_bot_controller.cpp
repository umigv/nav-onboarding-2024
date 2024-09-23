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
        "/orders", 10, std::bind(&PizzaBotController::order_callback, this, std::placeholders::_1));
    publisher_ = CREATE A PUBLISHER
    client_ = this->create_client<pizza_bot_interfaces::srv::NavigateToCoord>("navigate_to_coord");
    delivery_client_ = CREATE A CLIENT;
}

void PizzaBotController::order_callback(const pizza_bot_interfaces::msg::Order::SharedPtr msg)
{
       PUBLISH THE MSG TO THE RECIEVED_ORDER TOPIC ANDD CALL THE NAVIGATE FUNCTION
}

void PizzaBotController::navigate_to_coord(pizza_bot_interfaces::msg::Order::SharedPtr ord)
{
   
    auto request = std::make_shared<pizza_bot_interfaces::srv::NavigateToCoord::Request>();
    PUT THE RIGHT DATA INTO REQUEST
    const pizza_bot_interfaces::msg::Order::SharedPtr order = ord;
    std::function<void(rclcpp::Client<pizza_bot_interfaces::srv::NavigateToCoord>::SharedFuture)> cb = 
    std::bind(&PizzaBotController::pizza_navigation_callback,
            this,
            std::placeholders::_1,
            order) ;

    client_->async_send_request(
        request,
        cb);
            
  
}

void PizzaBotController::deliver_pizza(pizza_bot_interfaces::msg::Order::SharedPtr ord)
{

   SEND THE ASYNC REQUEST 
     
}
void PizzaBotController::delivery_bot_callback(FILL OUT){
     DONT NEED THIS FOR THIS PROJECT< YOU CAN JUST PRINT SOMETHING HERE

}


void PizzaBotController::pizza_navigation_callback(rclcpp::Client<pizza_bot_interfaces::srv::NavigateToCoord>::SharedFuture future,
pizza_bot_interfaces::msg::Order::SharedPtr ord)
{
    CALL THE DELIVER FUNCTION
}
