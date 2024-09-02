#include "restaurant.h"

#include <thread>

using namespace std::placeholders;
using MakePizza = pizza_bot_interfaces::action::MakePizza;
using MakePizzaGoalHandle = rclcpp_action::ServerGoalHandle<MakePizza>;

Restaurant::Restaurant()
    : Node("restaurant_node")
{
    declare_parameter("make_pizza_duration_seconds", 3);

    _pizza_action_server = rclcpp_action::create_server<MakePizza>(this,
        "make_pizza",
        std::bind(&Restaurant::handle_goal, this, _1, _2),
        std::bind(&Restaurant::handle_cancel, this, _1),
        std::bind(&Restaurant::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse Restaurant::handle_goal(const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MakePizza::Goal> goal)
{
    (void)uuid; // Prevent unused parameter warning

    RCLCPP_INFO(get_logger(),
        "Received request for a %s pizza",
        goal->pizza_type.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Restaurant::handle_cancel(const std::shared_ptr<MakePizzaGoalHandle> goal_handle)
{
    RCLCPP_INFO(get_logger(), 
        "Received request to cancel %s pizza",
        goal_handle->get_goal()->pizza_type.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Restaurant::handle_accepted(const std::shared_ptr<MakePizzaGoalHandle> goal_handle)
{
    // Execute action in new thread to prevent blocking the executor
    std::thread make_pizza_thread(std::bind(&Restaurant::make_pizza, this, _1),
        goal_handle);
    make_pizza_thread.detach();
}

void Restaurant::make_pizza(const std::shared_ptr<MakePizzaGoalHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    std::string pizza_type = goal->pizza_type;

    auto feedback = std::make_shared<MakePizza::Feedback>();
    feedback->pizza_status = pizza_type + " pizza in progress";
    goal_handle->publish_feedback(feedback);

    // Sleep to simulate making the pizza
    int make_pizza_duration_seconds = get_parameter("make_pizza_duration_seconds").as_int();
    rclcpp::sleep_for(std::chrono::seconds(make_pizza_duration_seconds));

    auto result = std::make_shared<MakePizza::Result>();
    result->pizza_finished = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(),
        "%s pizza complete",
        pizza_type.c_str());
}
