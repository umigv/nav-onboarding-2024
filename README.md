# Navigation Onboarding 2024
Welcome to the navigation sub-team! This onboarding project will help get you familiar with the Robot Operating System (ROS), an open-source collection of software libraries and tools used extensively in robotics application development. 

## ROS Overview
Useful links:\
[ROS Website](https://www.ros.org/)\
[ROS Docs](https://docs.ros.org/en/humble/)\
[C++ API](https://docs.ros2.org/latest/api/rclcpp/)

<img src="https://github.com/user-attachments/assets/4280dd47-33da-4135-a332-1eac12cce632" width="300">

Contrary to the name, the Robot Operating System is not an operating system at all, and while it was designed for robotics development, nothing about the core technology is exclusive for robotics. It is simply a framework that allows applications to be built in a particular way, which happens to be very useful for designing robots. Over the years, a massive ecosystem of robotics-specific libraries and tools have evolved around ROS, which is why it's so widely used today. 

The building blocks of ROS applications are nodes. Each node is a separate executable and can communicate with one another in three different ways: topics, services, and actions. This onboarding project will utilize all three of these communication methods. Nodes are then organized into packages, which is the format in which ROS applications are compiled and distributed. 

ROS provides both a Python API (rclpy) and a C++ API (rclcpp), but a node's implementation language is completely invisible to external users, meaning C++ and Python nodes can work together seamlessly. In the navigation sub-team, we primarily use the C++ API, so that is what you will be using for the onboarding project. 

ROS is designed for the Ubuntu Operating System, a Linux distribution, so some environment setup is required before diving into programming with ROS. 

## Environment Setup
You will be using pre-configured Ubuntu 22.04 virtual machines for your ROS environment. First, you need to download the virtualization software required to run these virtual machines, VMware Workstation Pro for Windows and VMware Fusion Pro for Mac. 

### Windows Instructions
- Download and run the installer found in the folder at the following link: https://drive.google.com/drive/u/1/folders/1B4brWb8zgHHhMUmMF9D1ZJPqDI5njiof
- When prompted, select “Install Windows Hypervisor Platform (WHP) automatically”
- Leave all other settings as the default
- Download and unzip the virtual machine found in the folder at the following link: TODO: insert link here
  - Note: the virtual machine itself will be a folder named "ARV ROS2 Humble AMD64 VM"
- Launch the VMware Workstation Pro application
- Select "Use VMware Workstation 17 for Personal Use"
- Select "Open a Virtual Machine"
- In the file explorer, navigate inside the unzipped virtual machine folder you downloaded and select the "VMware virtual machine configuration" file found inside
- This will open the pre-configured virtual machine, where a ROS workspace is already installed and ready to go
- The password to login is "arv123"

### Mac Instructions
- Download and run the installer found in the folder at the following link: https://drive.google.com/drive/u/1/folders/17qI6loxY2wwvchc0UcinaLhVEvrYvMe1
- Download and unzip the virtual machine found in the folder at the following link: TODO: insert link here
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

The infrastructure publishes orders very slowly, because eventually a lot of stuff will be happening between each order, so after about 15 seconds, if everything is working properly, an order message should pop up in the terminal you ran `ros2 topic echo /orders`, and there should be output in the other terminal saying that an order was published. 

Topic echo output:

<img src="https://github.com/user-attachments/assets/70f5129d-66b7-46a8-b866-b27c938b5c09" width="300">

Infrastructure output:

<img src="https://github.com/user-attachments/assets/04c3263b-32bd-4686-b797-abb7a39703f8" width="650">

Once you confirm that the infrastructure is functioning as expected, kill the process in each terminal with Ctrl+C. 

Now we're ready to start writing code! Run the following command to open the pizza_bot package in VSCode:
```bash
code ~/arv-ws/src/nav-onboarding-2024/pizza_bot
```

### Receiving the orders
