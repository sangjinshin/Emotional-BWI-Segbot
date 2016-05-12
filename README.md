# Emotional [BWI] (http://www.cs.utexas.edu/~larg/bwi_web/) Segway Robot
This repository contains a ROS package for CS378: Autonomous Intelligent Robotics (FRI) project. <br/>

Package Name: <br/>
emotion_driver

ROS Nodes: <br/>
emotion_driver <br/>
emotion_info <br/>
emotion_face

Summary: <br/>
This ROS package implements personality and emotional responses to the BWI Segway Robot.

#To Run: <br/>
0) Install the BWI code base. <br/>
https://github.com/utexas-bwi/bwi <br/>
1) In a new Terminal window,
```
rosrun segbot_bringup teleop_twist_keyboard
```
2) In a new Terminal window,
```
rosrun emotion_driver emotion_driver
```
3) In a new Terminal window,
```
rosrun emotion_driver emotion_info
```
4) In a new Terminal window,
```
rosrun emotion_driver emotion_face
```
