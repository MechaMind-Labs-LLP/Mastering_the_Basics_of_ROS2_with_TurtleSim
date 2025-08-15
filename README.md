# Mastering the Basics of ROS2 with TurtleSim 🐢

Welcome, future robotics pioneer\! This repository contains all the code, resources, and project files for the **Mastering the Basics of ROS2 with TurtleSim** course. Our mission is to take you from an absolute beginner to a confident ROS2 developer, capable of building and understanding real-world robotic applications.

We believe in learning by doing. Using the fun and intuitive TurtleSim simulator, you'll get hands-on experience with the fundamental concepts of ROS2 without the need for expensive hardware.

## 🚀 What You'll Learn: Your ROS2 Superpowers

By the end of this workshop, you will have the ability to:

  * ✅ **Think in ROS2:** Deconstruct complex robotics problems into a system of Nodes, Topics, and Services.
  * ✅ **Speak Robot:** Confidently use the ROS2 Command Line Interface (CLI) to inspect, debug, and interact with any ROS2 system.
  * ✅ **Build Robot Brains:** Write your own clean, efficient ROS2 nodes in Python (`rclpy`).
  * ✅ **Create Custom Tools:** Design and implement your own custom Messages and Services to build unique and powerful applications.
  * ✅ **Launch Complex Systems:** Use ROS2 launch files to start and configure multiple nodes at once.
  * ✅ **Tackle Projects:** Apply all your skills to complete a capstone project that solidifies your learning.

-----

## 🛠️ Part 1: Environment Setup & Installation

Before we can build robots, we need to build our workshop. This section will guide you through a complete installation of ROS2 Humble Hawksbill.

### **Step 1: System Requirements**

  * An installation of **Ubuntu 22.04 (Jammy Jellyfish)**, either on a physical machine or in a Virtual Machine.
  * Basic knowledge of the Linux Command Line (`cd`, `ls`, `mkdir`).
  * Basic knowledge of Python (variables, functions, classes).

### **Step 2: Install ROS2 Humble Hawksbill**

Open a terminal and run these commands step-by-step.

1.  **Set Locale:**

    ```bash
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ```

2.  **Add ROS2 Repository:**

    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

3.  **Install ROS2 Packages:**

    ```bash
    sudo apt update
    sudo apt install ros-humble-desktop ros-dev-tools -y
    ```

4.  **Source the ROS2 Environment:**

    ```bash
    source /opt/ros/humble/setup.bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```

### **Step 3: Set Up the Workshop Workspace**

1.  **Create a Workspace and Clone the Repository:**

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/mechamind-labs-llp/mastering_the_basics_of_ros2_with_turtlesim.git
    ```

2.  **Install Dependencies:**

    ```bash
    cd ~/ros2_ws
    sudo apt-get update
    rosdep install -i --from-path src -y --rosdistro humble
    ```

3.  **Build the Workspace:**

    ```bash
    cd ~/ros2_ws
    colcon build
    ```

4.  **Source Your Workspace:**

    ```bash
    source ~/ros2_ws/install/setup.bash
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    ```

Your development environment is now fully configured\!

-----

## 🤖 Part 2: Speaking Robot - Core ROS2 Concepts

Let's start by launching our simulator, **TurtleSim**.

```bash
ros2 run turtlesim turtlesim_node
```

A new window with a turtle will appear. Keep it open. Open a **new terminal** for the following commands.

### **Nodes: The Brains of the Operation**

A ROS2 system is a collection of processes called **nodes**. Each node has a single purpose. The `turtlesim_node` you just ran is a node responsible for the simulation.

  * **List running nodes:**

    ```bash
    ros2 node list
    ```

    You'll see `/turtlesim`.

  * **Get info about a node:**

    ```bash
    ros2 node info /turtlesim
    ```

    This shows everything the node publishes, subscribes to, services it offers, etc.

### **Topics: The Communication Channels**

Nodes exchange data on **topics**. A node can **publish** (send) messages to a topic, and any number of other nodes can **subscribe** (receive) messages from it.

  * **List all topics:**

    ```bash
    ros2 topic list
    ```

  * **Echo topic data:** Let's spy on the turtle's position data.

    ```bash
    ros2 topic echo /turtle1/pose
    ```

  * **Publish to a topic:** Let's send a command to make the turtle move in a circle.

    ```bash
    ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.0}}"
    ```

### **Messages: The Language of Topics**

Data is sent on topics via **messages**. Each topic has a specific message type.

  * **Find a topic's type:**

    ```bash
    ros2 topic info /turtle1/cmd_vel
    ```

    You'll see the type is `geometry_msgs/msg/Twist`.

  * **Show a message's structure:**

    ```bash
    ros2 interface show geometry_msgs/msg/Twist
    ```

    This shows the fields inside the message, which tells you how to structure the command you published earlier.

### **Services: The Question & Answer**

For request/response interactions, nodes use **services**. A **service client** sends a request, and a **service server** performs a task and returns a response.

  * **List all services:**

    ```bash
    ros2 service list
    ```

  * **Show a service's structure:**

    ```bash
    ros2 interface show turtlesim/srv/Spawn
    ```

    This shows the `request` fields (what you need to provide) and the `response` fields (what you get back).

  * **Call a service:** Let's create a new turtle.

    ```bash
    ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 8.0, name: 'turtle2'}"
    ```

    A new turtle will appear in the simulation\!

-----

## 📂 Part 3: Deep Dive into the Project Packages

This repository is structured into three key packages within the `src/` directory.

### **`turtle_msgs`**

This package defines the custom "language" for our project. It contains our custom message and service definitions.

  * `msg/AliveTurtle.msg`: A message to broadcast the position and name of a newly spawned turtle.
  * `msg/KillTurtle.msg`: A message to request that a specific turtle be killed by its name.
  * `srv/ComputeRectangleArea.srv`: A service definition that takes `length` and `width` and returns the `area`.

### **`turtle_script`**

This package contains all our Python nodes—the "brains" of our turtles. Each file is a different node with a specific job.

  * **`circle_drawer`**: Makes the turtle drive in an expanding spiral.
  * **`polygon_drawer`**: Draws a polygon with a configurable number of sides and length.
  * **`area_service_server` & `area_service_client`**: The server and client nodes for our custom area calculation service.
  * **`turtle_spawner` & `turtle_controller`**: The two main nodes for our "Catch 'em All" game.

### **`turtle_bringup`**

This package contains **launch files**, which allow us to start and configure multiple nodes with a single command.

  * `polygon_drawer.launch.py`: Starts the simulator and the polygon drawing node.
  * `catch_them_all.launch.py`: Starts the simulator and all nodes required for the game.

-----

## 🏆 Part 4: Running the Projects

Let's see our nodes in action\!

### **Project 1: The Area Service**

1.  **Terminal 1 (Run the Server):**

    ```bash
    ros2 run turtle_script area_service_server
    ```

2.  **Terminal 2 (Run the Client):**

    ```bash
    ros2 run turtle_script area_service_client
    ```

3.  **Run the Client with Parameters:**

    ```bash
    ros2 run turtle_script area_service_client --ros-args -p length:=12.5 -p width:=4.0
    ```

### **Project 2: The Polygon Drawer**

We will use the launch file, which makes this much simpler.

1.  **In a single terminal, run:**
    ```bash
    ros2 launch turtle_bringup polygon_drawer.launch.py
    ```
    This single command starts both the simulator and the drawing node. The turtle will begin drawing a 5-sided polygon with a side length of 2.5, as defined in the launch file.

### **Capstone Project: "Catch 'em All"**

This project showcases a complete ROS2 system with multiple nodes interacting through topics and services.

1.  **In a single terminal, run:**

    ```bash
    ros2 launch turtle_bringup catch_them_all.launch.py
    ```

    This will:

      * Start the TurtleSim window.
      * Start the **`turtle_spawner`**, which will begin adding new turtles at random locations.
      * Start the **`turtle_controller`**, which will identify the closest target turtle and begin chasing it.

    Watch as `turtle1` autonomously chases and "catches" the other turtles, which then disappear. This is a complete, miniature robotics application\!
