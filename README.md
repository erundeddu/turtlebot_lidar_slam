# ME495 Sensing, Navigation and Machine Learning For Robotics
* Edoardo Rundeddu
* Winter 2021
# Package List
This repository consists of several ROS packages
- nuturtle\_description - contains urdf files and basic debugging, testing, and visualization code for the robots used in ME495
- trect - makes a turtlesim turtle move along a rectangular path
- rigid2d - contains functions for performing 2D rigid body transformations, diff-drive robot definition, and ROS nodes for odometry and diff-drive robot kinematic simulation
- nuturtle\_robot - contains code to interact with the turtlebot hardware and run nodes on the turtlebot from a computer
- nurtlesim - simulation of robot kinematics and of landmark detection through a sensor
- nuslam - implements Feature-Based Kalman Filter SLAM with either known or unknown data association and visualizes estimated robot trajectory in rviz
