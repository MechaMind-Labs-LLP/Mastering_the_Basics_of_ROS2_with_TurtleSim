# 🐢 Mastering ROS 2 Basics with TurtleSim
---

## 👋 Welcome, Future Robotics Pioneer!

This repository is your **complete ROS 2 TurtleSim learning hub**, combining two powerful resources into one:

✅ **Mastering ROS 2 Basics with TurtleSim – A full hands-on course featuring:**

* Core ROS 2 concepts (**Nodes, Topics, Services, Messages, Parameters, RQT_Debugging**)
* **Circle Drawing with TurtleSim – A beginner-friendly project to get started with ROS 2 nodes, topics, and publishing commands.**
* **Polygon Drawer Project** (using launch files)
* **Capstone Project: Catch ‘em All** – A multi-node system where turtles spawn and your main turtle hunts them down!

By the end of this guide, you’ll:

* **Install and set up ROS 2 Humble** on Ubuntu
* **Learn ROS 2 CLI tools** and debug like a pro
* **Build Python nodes with `rclpy`**
* **Use custom messages & services**
* **Automate with launch files**
* **Complete advanced projects** for real-world robotics concepts

---

## 🚀 What You'll Learn
---

* ✔ **Think in ROS 2** – Break robotics problems into **Nodes**, **Topics**, and **Services**
* ✔ **Speak Robot** – Use **ROS 2 CLI** to inspect, debug, and interact with systems
* ✔ **Build Robot Brains** – Write efficient ROS 2 nodes in **Python (`rclpy`)**
* ✔ **Create Custom Tools** – Implement **custom Messages and Services**
* ✔ **Launch Complex Systems** – Start multiple nodes using **ROS 2 launch files**
* ✔ **Tackle Real Projects** – Draw shapes, control multiple turtles, and complete a **capstone project**

---

## 📦 Requirements

* **Ubuntu 22.04**
* [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* `colcon` build tool
* Python 3
* Basic knowledge of Linux CLI and Python

---

# 🛠️ Part 1: Environment Setup & Installation

### ✅ 1. Install ROS 2 Humble

```bash
sudo apt update && sudo apt upgrade
sudo apt install curl gnupg2 lsb-release locales software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Add ROS 2 repository and keys:

```bash
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install ROS 2 Desktop and tools:

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools python3-colcon-common-extensions python3-pip -y
```

Source ROS 2:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### ✅ 2. Install TurtleSim

```bash
sudo apt install ros-humble-turtlesim
```

Test it:

```bash
ros2 run turtlesim turtlesim_node
```

---

# 🐢 Part 2: Core ROS 2 Concepts – Speaking Robot

### **Nodes**

List nodes:

```bash
ros2 node list
```

Node info:

```bash
ros2 node info /turtlesim
```

---

### **Topics**

List topics:

```bash
ros2 topic list
```

Echo topic data:

```bash
ros2 topic echo /turtle1/pose
```

Publish to move turtle in a circle (CLI only):

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.0}}"
```

---

### **Messages**

Find topic type:

```bash
ros2 topic info /turtle1/cmd_vel
```

Show message structure:

```bash
ros2 interface show geometry_msgs/msg/Twist
```

---

### **Services**

List services:

```bash
ros2 service list
```

Find service type:

```bash
ros2 service type /spawn
```

Show service structure:

```bash
ros2 interface show turtlesim/srv/Spawn
```

Spawn a new turtle:

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 8.0, name: 'turtle2'}"
```

---

# 📂 Part 3: Project 1 – Draw a Circle in TurtleSim

### ✅ 1. Create ROS 2 Workspace and Package

```bash
mkdir -p ~/turtlesim_ws/src
cd ~/turtlesim_ws/src
ros2 pkg create --build-type ament_python turtlesim_circle
```

---

### ✅ 2. Add Your Python Node

Create `circle_drawer.py`:

```bash
mkdir -p turtlesim_circle/turtlesim_circle
touch turtlesim_circle/turtlesim_circle/circle_drawer.py
chmod +x turtlesim_circle/turtlesim_circle/circle_drawer.py
```

Add code:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_in_circle)

    def move_in_circle(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.publisher.publish(msg)
        self.get_logger().info("Drawing a circle...")

def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### ✅ 3. Update `package.xml`

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
```

---

### ✅ 4. Update `setup.py`

```python
from setuptools import setup
package_name = 'turtlesim_circle'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Draws a circle using turtlesim',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'draw_circle = turtlesim_circle.circle_drawer:main',
        ],
    },
)
```

---

### ✅ 5. Build and Run

```bash
cd ~/turtlesim_ws
colcon build
source install/setup.bash
```

Run nodes:

Terminal 1:

```bash
ros2 run turtlesim turtlesim_node
```

Terminal 2:

```bash
ros2 run turtlesim_circle draw_circle
```

The turtle will draw a circle 🐢.

---

# 🧩 Part 4: Advanced Projects & Launch Files

This section is from the **Mastering ROS 2 with TurtleSim** course.

### ✅ Polygon Drawer via Launch File

```bash
ros2 launch turtle_bringup polygon_drawer.launch.py
```

### ✅ Capstone Project – "Catch 'em All"

```bash
ros2 launch turtle_bringup catch_them_all.launch.py
```

**What happens:**

* Starts TurtleSim window
* Spawns turtles at random positions
* `turtle_controller` chases and "catches" them

---

# 📂 Final Project Structure

```
turtlesim_ws/
├── build/
├── install/
├── log/
├── src/
│   ├── turtle_script/ # Python nodes (circle, polygon, spawner)
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── turtle_script/
│   │       └── circle_drawer.py
│   ├── turtle_msgs/         # Custom messages & services     
│   └── turtle_bringup/      # Launch files
```

---

# ✅ Extra Commands for Debugging

* **List nodes**: `ros2 node list`
* **Node info**: `ros2 node info /circle_drawer`
* **List topics**: `ros2 topic list`
* **Echo topic**: `ros2 topic echo /turtle1/cmd_vel`
* **Show interface**: `ros2 interface show geometry_msgs/msg/Twist`
* **Visualization tool**: `rqt_graph`

---

# 🐍 Add to `.bashrc` for Convenience

```bash
echo "source ~/turtlesim_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

# 🧩 ROS 2 Aliases

```bash
alias cbs="colcon build --symlink-install"   # Build with symlink (for faster edits)
alias cb="colcon build"                      # Standard colcon build
alias sc="source ~/.bashrc"                  # Reload bash configuration
```

---

## 🎯 Summary

* **Beginner-friendly circle drawing node**
* **Core ROS 2 concepts (Nodes, Topics, Message, Services, Parameters)**
* **Advanced projects (Polygon Drawer & Catch-the-Turtles game)**
* **Launch files for automation**
