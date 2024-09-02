cmake_minimum_required(VERSION 3.8)
project(pizza_bot_infrastructure)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json 3.10.5 REQUIRED)
find_package(pizza_bot_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

# find header files in include
include_directories(include)

add_executable(
  order_publisher_node 
  src/run_order_publisher.cpp
  src/order_publisher.cpp
)
ament_target_dependencies(
  order_publisher_node
  rclcpp
  pizza_bot_interfaces
)

add_executable(
  customer_node
  src/run_customer.cpp
  src/customer.cpp
)
ament_target_dependencies(
  customer_node
  rclcpp
  pizza_bot_interfaces
)

add_executable(
  navigator_node
  src/run_navigator.cpp
  src/navigator.cpp
)
ament_target_dependencies(
  navigator_node
  rclcpp
  pizza_bot_interfaces
)

add_executable(
  restaurant_node
  src/run_restaurant.cpp
  src/restaurant.cpp
)
ament_target_dependencies(
  restaurant_node
  rclcpp
  rclcpp_action
  pizza_bot_interfaces
)

target_link_libraries(
  order_publisher_node 
  "${cpp_typesupport_target}"
  nlohmann_json::nlohmann_json
)

install(
  TARGETS
  order_publisher_node
  customer_node
  navigator_node
  restaurant_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()