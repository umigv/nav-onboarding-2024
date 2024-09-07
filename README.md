# Navigation Onboarding 2024
Welcome to the navigation sub-team! This onboarding project will help get you familiar with the Robot Operating System (ROS), an open-source collection of software libraries and tools used extensively in robotics application development. 

## ROS Overview
Useful links:\
[ROS Website](https://www.ros.org/)\
[ROS Docs](https://docs.ros.org/en/humble/)\
[C++ API](https://docs.ros2.org/latest/api/rclcpp/)

<img src="https://github.com/umigv/nav-onboarding-2024/blob/main/images/ROS%20Logo.png" width="300">

Contrary to the name, the Robot Operating System is not an operating system at all, and while it was designed for robotics development, nothing about the core technology is exclusive for robotics. It is simply a framework that allows applications to be built in a particular way, which happens to be very useful for designing robots. Over the years, a massive ecosystem of robotics-specific libraries and tools have evolved around ROS, which is why it's so widely used today. 

The building blocks of ROS applications are nodes. Each node is a separate executable and can communicate with one another in three different ways: topics, services, and actions. This onboarding project will utilize all three of these communication methods. Nodes are then organized into packages, which is the format in which ROS applications are compiled and distributed. 

ROS provides both a Python API (rclpy) and a C++ API (rclcpp), but a node's implementation language is completely invisible to external users, meaning C++ and Python nodes can work together seamlessly. In the navigation sub-team, we primarily use the C++ API, so that is what you will be using for the onboarding project. 

ROS is designed for the Ubuntu Operating System, a Linux distribution, so some environment setup is required before diving into programming with ROS. 

## Environment Setup
You will be using pre-configured Ubuntu 22.04 virtual machines for your ROS environment. First, you need to download the virtualization software required to run these virtual machines, VMware Workstation Pro for Windows and VMware Fusion Pro for Mac. 

### Windows Instructions
- Download and run the installer found in the folder at the following link: [Workstation Pro Installer](https://drive.google.com/drive/u/1/folders/1B4brWb8zgHHhMUmMF9D1ZJPqDI5njiof)
- When prompted, select “Install Windows Hypervisor Platform (WHP) automatically”
- Leave all other settings as the default
- Download and unzip the virtual machine found in the folder at the following link: [Virtual Machine](https://drive.google.com/drive/u/1/folders/1MHMFtIlwFGJZG5i-9k_x_0I_77PrtI__)
  - Note: the virtual machine itself will be a folder named "ARV ROS2 Humble AMD64 VM"
- Launch the VMware Workstation Pro application
- Select "Use VMware Workstation 17 for Personal Use"
- Select "Open a Virtual Machine"
- In the file explorer, navigate inside the unzipped virtual machine folder you downloaded and select the "VMware virtual machine configuration" file found inside
- This will open the pre-configured virtual machine, where a ROS workspace is already installed and ready to go
- The password to login is "arv123"

### Mac Instructions
- Download and run the installer found in the folder at the following link: [Fusion Pro Installer](https://drive.google.com/drive/u/1/folders/17qI6loxY2wwvchc0UcinaLhVEvrYvMe1)
- Download and unzip the virtual machine found in the folder at the following link: [Virtual Machine](https://drive.google.com/drive/u/1/folders/1AtrTcFdAR8XJhJ1agBp9nuoueDqO_DnV)
- Launch the VMware Fusion Pro application
- When prompted for a license key, select "I want to license VMware Fusion 13 Pro for Personal Use"
- In the upper menu, select File->Open and Run, and select the virtual machine you downloaded
- This will open the pre-configured virtual machine, where a ROS workspace is already installed and ready to go
- The password to login is "arv123"

And that's it for environment setup! You're ready to start the onboarding project.

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
git clone https://github.com/umigv/nav-onboarding-2024
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

The infrastructure publishes orders very slowly, because eventually a lot of stuff will be happening between each order, so after about 15 seconds, if everything is working properly, an order message should pop up in the terminal you ran `ros2 topic echo /orders`, and there should be output in the other terminal saying that an order was published. 

Topic echo output:

<img src="https://github.com/umigv/nav-onboarding-2024/blob/main/images/Topic%20echo%20output.png" width="400">

Infrastructure output:

<img src="https://github.com/umigv/nav-onboarding-2024/blob/main/images/Initial%20infra%20output.png" width="900">

Once you confirm that the infrastructure is functioning as expected, kill the process in each terminal with Ctrl+C. 

Run the following command to open the pizza_bot package in VSCode:
```bash
code ~/arv-ws/src/nav-onboarding-2024/pizza_bot
```

To get Intellisense to work, click on the Build button in the very bottom bar in VSCode and select a compiler from the drop-down menu. Output from CMake should appear in the OUTPUT window. It's ok if errors occured; it probably won't be able to find the pizza_bot_interfaces package, but Intellisense should still work. 

Now we're ready to start writing code! 

### Receiving the orders
You will be creating a single node in this project, which will be implemented as a single C++ class called PizzaBotController. Your code will go in 3 files, each of which are already created and found in the pizza_bot package: pizza_bot_controller.h, which will include class, member function, and member variable declarations, pizza_bot_controller.cpp, which will contain member function implementations, and main.cpp, which will contain the `main` function where your node is launched. 

Download the ROS2_Tutorial.pdf file found in this repo. It is an excellent introduction to ROS 2 concepts written by Chris Erndteman, our current engineering director and former navigation lead. This document will be referred to as the ROS2 Tutorial from now on. To get a conceptual understanding of ROS nodes, read section 4.1 of the ROS2 Tutorial and the Background section of the following article: [ROS Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html).

The first task is to subscribe to a topic to receive the orders. To get a conceptual understanding of topics, read section 4.2 of the ROS2 Tutorial and the Background section of the following article: [ROS Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

The name of the topic you need to subscribe to is "orders", and the message type is `pizza_bot_interfaces/msg/Order`. To look at what data is contained within these Order messages, you can run the following command: 
```bash
ros2 interface show pizza_bot_interfaces/msg/Order
```

Use the subscriber section of the following article as a reference when creating the subscriber: [ROS Publishers/Subscribers](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html).

Looking things up, researching, and reading documentation are perhaps the most useful skills you will need in ARV. Chat GPT can be a valuable resource as well. When you're ready to test your code, run `colcon build` (make sure to only do this in the arv_ws directory) to build your node. Make sure to run `source ~/arv_ws/install/setup.bash` in each new terminal you open and after building, or your changes won't have any effect. Run the PizzaBotController node with the following command: 
```bash
ros2 run pizza_bot pizza_bot_controller_node 
```

Open a new terminal window, ideally so you can see both at once, and launch the infrastructure using the same command you used before:
```bash
ros2 launch pizza_bot_infrastructure pizza_bot_infrastructure_launch.py
```

Remember to always run the pizza_bot_controller node before running the infrastructure to make sure your node doesn't miss any published orders. The infrastructure output won't be changed by your subscriber, so you can check that your subscriber is working by printing out the data in the Order message you receive and check that it matches the orders found in pizza_bot_infrastructure/config/orders.json. 

### Notify customer
The next task is to notify the customer that their order has been received. You will do this by publishing each order you receive to a topic called "received_orders". The message type of this topic is the same as the "orders" topic you subscribed to in the previous step, so you can publish the order exactly how you received it. 

Use the publisher section of the following article as a reference when creating the publisher: [ROS Publishers/Subscribers](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html). 

If you're publishing the orders correctly, the infrastructure output should look like this:

<img src="https://github.com/umigv/nav-onboarding-2024/blob/main/images/Order%20pub%20output.png" width="800">

### Navigate to the pizza place
The next task is to navigate the pizza bot to the correct pizza place. You will do this by calling a service provided by the infrastructure called "navigate_to_coord". To get a conceptual understanding of services, read section 4.3 of the ROS2 Tutorial and the Background section of the following article: [ROS Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html).

To look at the request and response types of the "navigate_to_coord" service, run the following command: 
```bash
ros2 interface show pizza_bot_interfaces/srv/NavigateToCoord
```

Each Order you received contains a pizza_place_coord that represents the location of the pizza restaurant, which you can use as the request when calling the service. The PizzaBotController node will be serving as the service client, so use the client node section of the following article as a reference when calling the service: [ROS Service/Client](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html).

The reference code found in the above article is all within the `main` function, so it will need to be adapted to our object-oriented approach. The client itself should be a member variable of the PizzaBotController class, and it should be created with `this->create_client()` in the constructor. 

Additionally, after calling `async_send_request`, which performs the service call, instead of spinning until the service completes, which is what the reference code does, we can pass a callback function to `async_send_request` as the second argument. This works exactly how you passed a callback function to `create_subscription`. For example:

```cpp
client->async_send_request(request, std::bind(&PizzaBotController::service_callback, this, std::placeholders::_1));
```

`PizzaBotController::service_callback` will be automatically called when the service completes. The first parameter of your service callback function must have the following type:

```cpp
rclcpp::Client<pizza_bot_interfaces::srv::NavigateToCoord>::SharedFuture future;
```

Inside the callback, you can get the result of the service like so: 
```cpp
bool success = future.get()->success;
```

Your callback can have more parameters if needed, but these parameters must be set with `std::bind` when passing it to `async_send_request`. 
_Hint: You may need to pass extra information about the order to your callback for future steps._

If you're calling the service correctly, the infrastucture output should look like this: 

<img src="https://github.com/umigv/nav-onboarding-2024/blob/main/images/Navigate%20to%20pizza%20place%20output.png" width="800">

As seen in the above screenshot, sometimes the customer_node and navigator_node output their messages in a different order, and that's completely fine; both nodes are completely independent processes running concurrently, so there will be some non-determinism in how they execute. 

### Retrieving the pizza
The next task is ordering and retrieving the pizza from the restaurant. You will do this by calling an action provided by the infrastructure called "make_pizza". To get a conceptual understanding of actions, read section 4.5 of the ROS2 Tutorial and the Background section of the following article: [ROS Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html).

To look at the data types of the "make_pizza" action, run the following command: 

```bash
ros2 interface show pizza_bot_interfaces/action/MakePizza
```

Use the action client node section of the following article as a reference when calling the action: [ROS Action Server/Client](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html#writing-an-action-client). The reference code here takes an object-oriented approach, so you can follow it more closely than you did for the service client node example. 

The feedback of the "make_pizza" action doesn't have a functional purpose, but you can print it to make sure that your feedback callback is working correctly. 

If you're calling the action correctly for each order, the infrastructure output should look like this:

<img src="https://github.com/umigv/nav-onboarding-2024/blob/main/images/Deliver%20pizza%20output.png" width="800">

### Navigate to the customer's house
The next task is to navigate the pizza robot to the customer's house. You will do this with the same service you used to navigate to the pizza place, "navigate_to_coord", which means you can use the same client object you created a couple steps ago. Try to reuse as much code as possible from when you last called the service. Each Order contains a customer_coord that has the coordinates of the customer's house. 

If you're navigating to the customer's house correctly, the infrastructure output should look like this: 

<img src="https://github.com/umigv/nav-onboarding-2024/blob/main/images/Navigate%20to%20customer%20output.png" width="800">

### Deliver the pizza
The final task is to deliver the pizza to the customer. You will do this by calling a different service provided by the infrastructure called "deliver_pizza". To look at the data types of the "deliver_pizza" service, run the following command: 

```bash
ros2 interface show pizza_bot_interfaces/srv/DeliverPizza 
```

Use the same code structure as you did when calling the "navigate_to_coord" service. This will be the final task for each order, so after delivering the pizza to the customer, simply wait for the next order to be published and repeat the sequence. After completing this step, the final infrastructure output should look like this: 

<img src="https://github.com/umigv/nav-onboarding-2024/blob/main/images/Deliver%20pizza%20output.png" width="800">

And that's it! After completing this onboarding project, you should have a solid understanding of the basic structure of a ROS application and the three primary methods nodes use to communicate with each other. When you're finished, let a lead know and we'll start talking about what project you might want to work on!
