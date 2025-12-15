ROS 2 Fundamentals
The Foundation of Modern Robotics Communication

Robots are complex systems composed of sensors, actuators, controllers, and AI models. To coordinate all these components, they need a powerful communication framework. ROS 2 (Robot Operating System 2) is that framework — the nervous system that allows robots to sense, think, and act seamlessly.

This chapter introduces the essential concepts of ROS 2 and builds the foundation for humanoid robotics and Physical AI.

🌐 1. Introduction to ROS 2

ROS 2 is a distributed middleware designed for real-time, reliable, and modular robotics development.
It enables multiple processes (nodes) to communicate across networks, computers, or even microcontrollers.

Why ROS 2?

Built for real-time robotic control

Highly scalable and distributed

Strong cross-platform support (Linux, Windows, macOS)

Designed for industrial-grade robots

Backed by a massive open-source ecosystem

ROS 2 is used in NASA robots, autonomous vehicles, industrial arms, drones, quadrupeds, and humanoids.

🔧 2. ROS 2 Architecture Overview

At the core of ROS 2 is the DCPS/DDS communication standard — a fast, reliable messaging system used in aerospace and automotive systems.

ROS 2 architecture includes:

Nodes

Topics

Services

Actions

Parameters

Launch Files

Packages

Each piece plays a specific role in the robot’s communication ecosystem.

🧩 3. Core ROS 2 Concepts
3.1 Nodes — The Building Blocks of a Robot

A node is a single-purpose program that performs a specific function.

Examples of nodes:

/camera_node → publishes camera frames

/lidar_node → publishes point clouds

/controller_node → moves motors

/slam_node → maps the environment

Nodes can run on:

the main robot PC,

a Jetson edge device,

or an external workstation.

3.2 Topics — Continuous Data Streams

Topics are ROS 2 communication channels used for streaming data.

A topic has:

a publisher (sends data)

a subscriber (receives data)

Example topics:

/camera/image_raw

/scan

/imu/data

/cmd_vel

Used for:

sensor streams

velocity commands

real-time feedback

3.3 Services — Request/Response Communication

Services are used when a node needs to ask another node a question.

Example:

Request: "Reset the IMU."
Response: "IMU reset successful."


Use services when:

A response is expected

The task is quick

3.4 Actions — Long-Running Tasks with Feedback

Actions are for tasks that take time and require updates.

Examples:

Navigate to a waypoint

Pick up an object

Follow a trajectory

Actions provide:

feedback

progress

completion status

cancellation

3.5 Parameters — Robot Configuration Settings

Parameters store configuration values that can be changed without editing code.

Examples:

PID gains

camera resolution

robot speed limits

sensor thresholds

3.6 Launch Files — Running Multiple Nodes Together

Launch files allow you to start several nodes at once.

Uses:

bring up robot sensors

launch navigation stack

spawn URDF model in Gazebo

Launch files support:

Python

XML

YAML

Example:

ros2 launch my_robot bringup.launch.py

🦾 4. ROS 2 Packages

A package is the fundamental unit of software in ROS 2.

A package contains:

source code

configuration files

launch files

URDF models

dependencies

Every ROS 2 project you create is a package.

📡 5. Communication in ROS 2 (DDS Layer)

ROS 2 uses Data Distribution Service (DDS) under the hood.

Benefits:

extremely low latency

real-time communication

security features

automatic discovery of nodes

reliable message delivery

DDS allows ROS 2 robots to scale from tiny microcontrollers to large distributed fleets.

🔍 6. Debugging & Visualization Tools
Rviz2

3D visualization tool for:

robot models

camera streams

laser scans

SLAM maps

Rqt

GUI dashboard for:

graphs

parameters

node connections

ros2 topic / service / action tools

Command-line tools to inspect and interact with ROS communication.

⚙️ 7. Hands-On: Your First ROS 2 Graph

A simple robot usually includes:

camera_node

slam_node

navigation_node

controller_node

And they communicate like this:

Camera → SLAM → Navigation → Controller → Motors


This pipeline forms the basis of robotic autonomy.

🚀 8. Summary

ROS 2 Fundamentals equips you with the essential knowledge required to understand how robots communicate, coordinate, and function. By mastering nodes, topics, services, actions, and packages, you gain the foundation to work with humanoid robots, AI-driven perception systems, and real-world robotic applications.

This knowledge is the first major step toward building intelligent Physical AI systems.