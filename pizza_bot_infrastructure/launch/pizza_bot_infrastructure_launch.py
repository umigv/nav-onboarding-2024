import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Launches necessary nodes from pizza_bot_infrastructure package
# order_publisher parameters are stored in config/order_publisher_params.yaml

def generate_launch_description():
    pizza_bot_infrastructure_config = os.path.join(get_package_share_directory('pizza_bot_infrastructure'),
        'config')
    
    order_publisher_params = os.path.join(pizza_bot_infrastructure_config,
        'order_publisher_params.yaml')
    navigator_params = os.path.join(pizza_bot_infrastructure_config,
        'navigator_params.yaml')
    restaurant_params = os.path.join(pizza_bot_infrastructure_config,
        'restaurant_params.yaml')

    order_publisher_node = Node(package = 'pizza_bot_infrastructure',
        name = 'order_publisher_node',
        executable = 'order_publisher_node',
        parameters = [order_publisher_params])

    navigator_node = Node(package = 'pizza_bot_infrastructure',
        name = 'navigator_node',
        executable = 'navigator_node',
        parameters = [navigator_params])
    
    customer_node = Node(package = 'pizza_bot_infrastructure',
        name = 'customer_node',
        executable = 'customer_node')
    
    restaurant_node = Node(package = 'pizza_bot_infrastructure',
        name = 'restaurant_node',
        executable = 'restaurant_node',
        parameters = [restaurant_params])
    
    nodes = [order_publisher_node,
        navigator_node,
        customer_node,
        restaurant_node]

    return LaunchDescription(nodes)