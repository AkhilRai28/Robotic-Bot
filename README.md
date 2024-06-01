## Auto-and-Tele-Operated Bot

## Project Overview

This project involves the development of a teleoperated and autonomous robot using the Gazebo ROS framework. The robot is capable of being controlled remotely and can also operate autonomously, interacting with both simulated and physical hardware using keyboard inputs.

## Outcomes

1. Implemented and simulated the Goal-To-Goal algorithm in Turtlesim.
2. Programmed and implemented an autonomous algorithm on hardware using ultrasonic sensors.

## Table of Contents

1. [Dual Booting](#dual-booting)
2. [ROS Installation and Terminologies](#ros-installation-and-terminologies)
3. [ROS Commands](#ros-commands)
4. [Publisher-Subscriber Files](#publisher-subscriber-files)
5. [Turtlesim](#turtlesim)
6. [Rosserial - Arduino](#rosserial---arduino)
7. [Hardware](#hardware)
8. [Teleoperation in Turtlesim](#teleoperation-in-turtlesim)
9. [Goal-To-Goal](#goal-to-goal)
10. [PID Based Autonomous Algorithm](#pid-based-autonomous-algorithm)
11. [Vector Based Autonomous Algorithm](#vector-based-autonomous-algorithm)
12. [Errors Encountered](#errors-encountered)
13. [Bibliography](#bibliography)

## Dual Booting

- **Setup**: Installed Ubuntu 20.04 alongside an existing operating system (typically Windows) to enable a dual-boot configuration.
- **Tools Used**: Used Rufus 4.1 for creating a bootable USB drive with the Ubuntu installation ISO.
- **Purpose**: This setup was necessary to run ROS (Robot Operating System) which is best supported on Ubuntu.

## ROS Installation and Terminologies

- **Installation**: Followed official ROS Noetic installation guidelines for Ubuntu 20.04, setting up the environment with essential packages and dependencies.
- **Key Concepts**:
  - **Nodes**: Independent processes that perform computation.
  - **Topics**: Channels over which nodes exchange messages.
  - **Packages**: Collections of nodes, data, and configuration files.
  - **Workspaces**: Directories where ROS packages are stored and managed.

## ROS Commands

- **Basic Commands**:
  - `roscore`: Starts the ROS master, which coordinates communication between nodes.
  - `rosnode`: Manages nodes; commands include `rosnode list` and `rosnode info`.
  - `rosrun`: Runs a node from a specified package.
  - `rostopic`: Manages topics; commands include `rostopic list` and `rostopic echo`.
  - `roslaunch`: Launches multiple nodes and configurations from a single file.

## Publisher-Subscriber Files

- **Publisher**: A ROS node that sends data to a topic.
- **Subscriber**: A ROS node that receives data from a topic.
- **Implementation**: Created ROS nodes that publish and subscribe to topics to facilitate communication between different parts of the system, such as sensor data from hardware to a control algorithm.

## Turtlesim

- **Learning Tool**: Turtlesim is a ROS package that provides a simple simulator for learning and testing basic ROS concepts.
- **Tasks**:
  - **Movement**: Controlled the turtle to move in different patterns (e.g., circles, rectangles).
  - **Algorithms**: Implemented basic navigation algorithms to move the turtle to specified goals.

## Rosserial - Arduino

- **Integration**: Used rosserial to connect an Arduino microcontroller with the ROS ecosystem, allowing the Arduino to communicate with ROS nodes.
- **Functionality**: Enabled the robot to process sensor data and control motors based on commands received via ROS.

## Hardware

- **Components**:
  - **Ultrasonic Sensors (HCSR04)**: Used for distance measurement and obstacle detection.
  - **Cytron Motor Drivers**: Controlled the speed and direction of motors.
  - **Arduino UNO**: Microcontroller for processing sensor data and controlling motors.
- **Assembly**: Detailed steps for assembling the robot hardware and connecting the components to the Arduino.

## Teleoperation in Turtlesim

- **Mobile App**: Used a ROS-compatible mobile application to control the Turtlesim turtle remotely via a joystick interface.
- **Functionality**: Implemented teleoperation to demonstrate manual control of the robot using keyboard or joystick inputs.

## Goal-To-Goal

- **Algorithm**: Developed an algorithm to navigate the turtle from one goal point to another.
- **Messages**:
  - **Twist**: Command velocity (linear and angular).
  - **Pose**: Position and orientation of the turtle.
- **Implementation**: Tested the algorithm in Turtlesim and then on the hardware.

## PID Based Autonomous Algorithm

- **PID Controller**: Implemented a Proportional-Integral-Derivative (PID) controller to maintain the desired trajectory and speed while avoiding obstacles.
- **Functionality**: Ensured smooth and stable navigation by adjusting control inputs based on sensor feedback.

## Vector Based Autonomous Algorithm

- **Algorithm**: Developed a vector-based algorithm to navigate the robot by calculating resultant vectors from multiple sensors.
- **Functionality**: Enhanced obstacle avoidance by directing the robot away from detected obstacles and towards the goal.

## Errors Encountered

- **Documentation**: Recorded various issues faced during development, including software bugs, hardware malfunctions, and integration challenges.
- **Solutions**: Provided troubleshooting steps and solutions for each error encountered.

## Bibliography

- [ROS Official Website](https://www.ros.org/)
- [What is ROS? - Ubuntu Robotics](https://ubuntu.com/robotics/what-is-ros)
- [Dual Boot Guide - FreeCodeCamp](https://www.freecodecamp.org/news/how-to-dual-boot-any-linux-distribution-with-windows/)
- [ROS Commands Summary - Packt](https://subscription.packtpub.com/book/iot-and-hardware/9781788479592/1/ch01lvl1sec15/ros-commands-summary)
