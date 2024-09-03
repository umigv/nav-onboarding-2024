# Navigation Onboarding 2024
Welcome to the navigation sub-team! This onboarding project will help get you familiar with the Robot Operating System (ROS), an open-source collection of software libraries and tools used extensively in robotics application development. 

## ROS Overview
Useful links:\
[ROS Website](https://www.ros.org/)\
[ROS Docs](https://docs.ros.org/en/humble/)\
[C++ API](https://docs.ros2.org/latest/api/rclcpp/)

Contrary to the name, the Robot Operating System is not an operating system at all, and while it was designed for robotics development, nothing about the core technology is exclusive for robotics. It is simply a framework that allows applications to be built in a particular way, which happens to be very useful for designing robots. Over the years, a massive ecosystem of robotics-specific libraries and tools have evolved around ROS, which is why it's so widely used today. 

The building blocks of ROS applications are nodes. Each node is a separate executable and can communicate with one another in three different ways: topics, services, and actions. This onboarding project will utilize all three of these communication methods. Nodes are then organized into packages, which is the format in which ROS applications are compiled and distributed. 

ROS provides both a Python API (rclpy) and a C++ API (rclcpp), but a node's implementation language is completely invisible to external users, meaning C++ and Python nodes can work together seamlessly. In the navigation sub-team, we primarily use the C++ API, so that is what you will be using for the onboarding project. 

ROS is designed for the Ubuntu Operating System, a Linux distribution, so some environment setup is required before diving into programming with ROS. 

## Environment Setup



## Pizza Delivery Robot
You will be designing a pizza delivery robot control system that handles order processing, navigation, pizza pickup, and delivery. You will need to complete the following actions for each order:  

- Receive and process the order
- Notify the customer that the order has been received
- Navigate to the pizza place
- Retrieve the pizza
- Navigate to the customer's house
- Deliver the pizza

In ROS terms:

- Subscribe to the orders topic
- Publish a message to the received_orders topic
- Call the navigate_to_coord service
- Call the make_pizza action
- Call the navigate_to_coord service again
- Call the deliver_pizza service

### Initial Setup
To begin, open a terminal window from the sidebar on the left, navigate to the workspace src directory, and clone this repo:
``` bash
cd ~/arv_ws/src
git clone https://github.com/johnr282/nav-onboarding-2024 ### TODO: Update this link once repo is moved into umigv
```

If you look in your src directory, you will see three packages: pizza_bot, pizza_bot_infrastructure, and pizza_bot_interfaces. All of the code you will be writing will go in the pizza_bot package. Do not edit any files in the pizza_bot_infrastructure or pizza_bot_interfaces packages, but feel free to look inside them if you're curious. 

Now, open another terminal tab, navigate back to the workspace directory, and build the code you just cloned:
``` bash
cd ~/arv_ws
colcon build
```

Colcon is the ROS build tool used to compile all of the packages in the src directory. 

Now we need to make sure that the infrastructure is working properly. Open another terminal tab and a separate terminal window so two terminal windows are visible at once (get used to opening lots and lots of terminal windows/tabs). Make sure you're in the workspace directory (~/arv_ws) in each new terminal. 

**_Important: After opening a new terminal or after running `colcon build`, you need to run the following command to sync your workspace with the underlying ROS installation:_**
```bash
source ~/arv_ws/install/setup.bash
# If you're already in the arv_ws directory, you can simply run:
source install/setup.bash
```

**_If you forget to do this, you will not be able to run any of the code in your src directory. Any time you are getting a weird error that you don't understand, this is the first thing to try._**

Note: It is advised to not run `source install/setup.bash` in the same terminal that you run `colcon build`, so get in the habit of having a dedicated terminal tab for building that you don't run anything in. 

Run `source install/setup.bash` in each of the two terminal windows you opened. We will now be able to run the infrastructure code we just built. In one terminal, run:
```bash
ros2 launch pizza_bot_infrastructure pizza_bot_infrastructure_launch.py
```

In the other, run:
```bash
ros2 topic echo /orders
```

The infrastructure publishes orders very slowly, because eventually a lot of stuff will be happening between each order, so after about 15 seconds, if everything is working properly, an order message should pop up in the terminal you ran `ros2 topic echo /orders`, and there should be output in the other terminal saying that an order was published. Once you confirm that the infrastructure is functioning as expected, kill the process in each terminal with Ctrl+C. Now we're ready to start writing code!

### Receiving the orders
