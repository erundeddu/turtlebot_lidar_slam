# Feature-Based Kalman Filter SLAM and Feature Detection through Circle regression
* Implementation of the Feature-Based Kalman Filter SLAM
* Visualization of SLAM estimated robot trajectory (blue) and odometry-only trajectory (red) in rviz
* Visualization of SLAM estimated landmarks (blue) in rviz
* Compatible with nurtlesim simulation or with real turtlebot
* Detection of obstacle tubes from radar scan data through regression
# Example Usage
```
roslaunch nuslam slam.launch robot:=localhost
```
![Sample simulation](images/sim1.png)
* Green path: the real path of the simulated robot
* Blue path: the estimated path through SLAM, approximately matches the real path
* Red path: the estimated path through odometry only, does not match the real path
* Turtle frame: the real pose of the robot
* Map frame to odom frame: the corrections to the pose estimate through SLAM
* Green markers: the real position of the obstacles in space
* Red markers: the current position of the obstacles in space sensed by the robot (far obstacles do not have red overlap since they may be outside of the sensing range)
* Dark blue markers: the position of the obstacles in space estimated through SLAM

![Collision simulation](images/sim2.png)
* Shows approximately matching real path and SLAM path at the beginning of the simulation, while the odometry path is different due to the robot wheels moving during the collision

```
roslaunch nuslam landmark_detect.launch robot:=localhost
```

