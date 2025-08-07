# Mastering the Basics of ROS2 with TurtleSim

\<p align="center"\>
\<img src="[https://img.shields.io/badge/ROS2-Humble-blue](https://www.google.com/search?q=https://img.shields.io/badge/ROS2-Humble-blue)" alt="ROS2 Humble"\>
\<img src="[https://img.shields.io/badge/Platform-Ubuntu\_22.04-orange](https://www.google.com/search?q=https://img.shields.io/badge/Platform-Ubuntu_22.04-orange)" alt="Ubuntu 22.04"\>
\<img src="[https://img.shields.io/badge/Language-Python-yellow](https://www.google.com/search?q=https://img.shields.io/badge/Language-Python-yellow)" alt="Python"\>
\<img src="[https://img.shields.io/badge/License-Apache\_2.0-lightgrey](https://www.google.com/search?q=https://img.shields.io/badge/License-Apache_2.0-lightgrey)" alt="License"\>
\</p\>

Welcome, future robotics pioneer\! This repository contains all the code, resources, and project files for the **Mastering the Basics of ROS2 with TurtleSim** course. Our mission is to take you from an absolute beginner to a confident ROS2 developer, capable of building and understanding real-world robotic applications.

We believe in learning by doing. Using the fun and intuitive TurtleSim simulator, you'll get hands-on experience with the fundamental concepts of ROS2 without the need for expensive hardware.

-----

## 🚀 What You'll Learn: Your ROS2 Superpowers

By the end of this course, you will have the ability to:

  * ✅ **Think in ROS2:** Deconstruct complex robotics problems into a system of Nodes, Topics, and Services.
  * ✅ **Speak Robot:** Confidently use the ROS2 Command Line Interface (CLI) to inspect, debug, and interact with any ROS2 system.
  * ✅ **Build Robot Brains:** Write your own clean, efficient ROS2 nodes in Python (`rclpy`).
  * ✅ **Create Custom Tools:** Design and implement your own custom Messages and Services to build unique and powerful applications.
  * ✅ **Tackle Projects:** Apply all your skills to complete a capstone project that solidifies your learning.
  * ✅ **Launch Your Career:** Possess the foundational knowledge that top robotics companies look for.

## 📋 Prerequisites

To get the most out of this course, you should have:

  * **Software:**
      * Ubuntu 22.04 (Jammy Jellyfish) installed on a PC/laptop or in a Virtual Machine.
      * ROS2 Humble Hawksbill installed. (We cover this in Module 1).
  * **Knowledge:**
      * **Basic Linux Command Line:** You should be comfortable with `cd`, `ls`, `mkdir`, `cp`, and `gedit` or another text editor.
      * **Basic Python:** You should understand variables, data types, loops, functions, and classes. The course will provide all the code, but a basic understanding will help you immensely.

## 📚 Course Structure & Syllabus

The course is divided into 5 modules, designed to take you on a logical journey from theory to practice.

| Module | Title                                        | Key Topics Covered                                                                                             |
| :----: | -------------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| **1** | **Foundations of ROS2** | Why ROS2?, Core Philosophy, Installation, Environment Setup (`.bashrc`), Introduction to TurtleSim.                |
| **2** | **Speaking Robot - Core Concepts via CLI** | Nodes, Topics, Messages, Services, Parameters. Controlling TurtleSim entirely from the command line, `rqt_graph`. |
| **3** | **Becoming a Builder - Your First Program** | Creating a ROS2 Workspace & Package, Writing a Python Node (`rclpy`), Publishers, Timers, The Build & Run Cycle. |
| **4** | **Advanced Development & Customization** | Creating Custom Messages (`.msg`), Custom Services (`.srv`), Implementing Service Servers & Clients.            |
| **5** | **Projects & Your Future in Robotics** | Capstone Project ("Catch 'em All"), Challenge Project ("Don't Follow Me"), Course Wrap-up, Next Steps.         |

## 🛠️ Setup & Installation

Follow these steps to get your environment ready to run the code in this repository.

1.  **Install ROS2 Humble:** If you haven't already, please follow the [Official ROS2 Humble Installation Guide](https://www.google.com/search?q=https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

2.  **Clone this Repository:** Open a terminal and clone this repository into a new ROS2 workspace directory.

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone <URL_OF_THIS_REPOSITORY>
    ```

3.  **Install Dependencies:** `rosdep` is a tool that automatically installs system dependencies for the packages in your workspace.

    ```bash
    cd ~/ros2_ws
    sudo apt-get update
    rosdep install -i --from-path src -y --rosdistro humble
    ```

4.  **Build the Workspace:** Use `colcon` to build all the packages.

    ```bash
    cd ~/ros2_ws
    colcon build
    ```

5.  **Source the Workspace:** Each time you open a new terminal to work with this project, you must source the workspace's setup file. This adds your packages to the ROS2 environment.

    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ```

    > **Pro-Tip:** Add `source ~/ros2_ws/install/setup.bash` to the end of your `~/.bashrc` file to do this automatically for every new terminal\!

## 📂 How to Use This Repository

This repository is structured to follow the course modules.

  * `src/`: This directory contains all the ROS2 packages.
      * `my_turtle_controller/`: The main package we develop throughout Modules 3 and 4. It contains the code for moving the turtle in circles, spirals, and polygons.
      * `catch_em_all_project/`: The solution package for the Module 5 capstone project.
      * `dont_follow_me_project/`: The solution package for the Module 5 challenge project.

It is recommended to code along with the video lessons. However, if you get stuck, you can always refer to the code in these packages for a complete solution.

## 🏆 Projects Showcase

### Capstone Project: "Catch 'em All"

Apply your knowledge of topics, services, and parameters to create a fun game where a "hunter" turtle automatically seeks out and "catches" randomly spawning target turtles.

### Challenge Project: "Don't Follow Me"

Integrate an external package (`teleop_twist_keyboard`) and write a control node to make one turtle autonomously follow another while you drive the leader around.

## 👨‍🏫 About the Instructor

**[Your Name]**

I am a [Your Title] with a passion for making robotics accessible to everyone. With [X] years of experience in the field, my goal is to provide you with the authentic, high-impact robotics content you need to succeed.

  * **LinkedIn:** [Your LinkedIn URL]
  * **Website:** [Your Website URL]

## 🤝 Contributing & Community

This repository is primarily for educational purposes, but contributions and suggestions are welcome\!

  * **Bugs & Issues:** If you find a bug in the code or an error in the documentation, please [open an issue](https://www.google.com/search?q=https://github.com/your_username/your_repo/issues).
  * **Questions:** For general questions about the course content, please use the course's official Q\&A forum or Discord channel.

## 📄 License

This project is licensed under the Apache 2.0 License. See the [LICENSE](https://www.google.com/search?q=LICENSE) file for details.
