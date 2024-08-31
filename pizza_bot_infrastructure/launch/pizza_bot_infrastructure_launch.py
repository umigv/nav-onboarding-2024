import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Launchs order_publisher_node and customer_node from pizza_bot_infrastructure package
# order_publisher parameters are stored in config/order_publisher_params.yaml

def generate_launch_description():

    order_publisher_params = os.path.join(get_package_share_directory('pizza_bot_infrastructure'),
        'config',
        'order_publisher_params.yaml')

    order_publisher_node = Node(package = 'pizza_bot_infrastructure',
        name = 'order_publisher_node',
        executable = 'order_publisher_node',
        parameters = [order_publisher_params])

    customer_node = Node(package = 'pizza_bot_infrastructure',
        name = 'customer_node',
        executable = 'customer_node')
    
    nodes = [order_publisher_node,
        customer_node]

    return LaunchDescription(nodes)