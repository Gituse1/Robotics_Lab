Lab 1: 4-Wheeled Autonomous Robot in Gazebo Harmonic
Overview
This project contains a completed simulation of a custom 4-wheeled mobile robot built using SDFormat (SDF) 1.8. The robot is equipped with a differential drive system and a LiDAR sensor for obstacle detection.

The project was developed as part of a robotics laboratory work to master robot modeling, physical properties, and sensor integration in the Gazebo simulation environment.

Project Structure
worlds/robot.sdf: The main simulation file containing the world environment, 3D models, and physics plugins.

Architecture:

Symmetric 4-wheel chassis.

Independent revolute joints for each wheel.

Fixed-joint LiDAR mount.

Key Features
1. 4-Wheel Differential Drive
The robot uses the gz-sim-diff-drive-system plugin. Unlike standard tutorials, this model is configured for 4 active joints, ensuring better traction and stability.

Topic: /cmd_vel

Joints: left_front, left_rear, right_front, right_rear

2. LiDAR Sensing
A GPU-based LiDAR is integrated for real-time environment perception.

Range: 0.08m to 10.0m

FOV: ~80° (1.39 rad)

Topic: /lidar

3. Physics & Collision
All components have calculated inertial properties (mass and inertia matrices) for realistic simulation.

The world includes various obstacles (cylinders and boxes) to test sensor data and collision handling.

How to Run
1. Launch the Simulation
Open your terminal and enter the Docker container:

Bash
./scripts/cmd bash
Then launch the Gazebo world:

Bash
gz sim /opt/ws/src/code/lab1/worlds/robot.sdf
2. Control the Robot
In a new terminal (inside the container), send a movement command:

Bash
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.2}"
3. Monitor Sensor Data
To see the LiDAR output (distance to obstacles), use:

Bash
gz topic -e -t /lidar




Це чудове рішення. Твій перший README.md написаний дуже професійно, тому ми витримаємо такий самий стиль для другої частини. Оскільки тепер проект перейшов від чистої симуляції в Gazebo до повноцінної екосистеми ROS 2, нам потрібно акцентувати увагу на вузлах (nodes) та мостах (bridges).

Ось доповнення, яке ти можеш вставити відразу після секції першої лабораторної:

Lab 2: ROS 2 Integration, Sensors, and Autonomous Control
Overview
This stage of the project integrates the previously developed robot model with the ROS 2 Jazzy framework. The goal was to establish a communication bridge between the physics engine (Gazebo) and ROS 2 nodes to process sensor data and control the robot programmatically using Python.

Project Structure
lab2/package.xml: ROS 2 package metadata and dependencies (rclpy, sensor_msgs, geometry_msgs).

lab2/setup.py: Configuration for Python entry points.

lab2/lab2/robot_controller.py: A ROS 2 node that publishes movement commands.

lab2/lab2/lidar_subscriber.py: A ROS 2 node that processes LiDAR data and detects obstacles.

lab2/launch/gazebo_ros2.launch.py: Unified launch file for Gazebo, RViz2, and the ROS-GZ bridge.

Key Features
1. ROS-GZ Bridge Integration
A bidirectional bridge (ros_gz_bridge) was implemented to translate messages between Gazebo and ROS 2:

Command Bridge: Translates geometry_msgs/Twist from ROS 2 to Gazebo for movement.

Sensor Bridge: Translates gz.msgs.LaserScan to sensor_msgs/msg/LaserScan for ROS 2 processing.

2. Autonomous Python Controller
Logic: Implements a smooth "zigzag" movement pattern using a sine wave function for angular velocity.

Topic: Publishes to /cmd_vel.

3. LiDAR Data Processing
Logic: Subscribes to the /lidar topic, calculates the minimum distance to surrounding objects, and logs warnings if obstacles are detected.

Visualization: Fully integrated with RViz2 for real-time laser scan rendering.

How to Run
1. Build the Workspace
Inside the Docker container (/opt/ws):

Bash
colcon build --packages-select lab2 --symlink-install
source install/setup.bash
2. Launch Simulation & Visualization
Bash
ros2 launch lab2 gazebo_ros2.launch.py
3. Start Robot Nodes
In separate terminals:

To move the robot: ros2 run lab2 robot_controller

To monitor LiDAR: ros2 run lab2 lidar_subscriber