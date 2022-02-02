# simple_motion_planning_demo
A simple motion planning demo using the RRT planning algorith, Forward and Inverse Kinematics implemantation, custom trajectory exection using interpolation and sending joint positions to the arm.

Makes use of the https://github.com/ros-industrial/kuka_experimental for the Kuka kr210l150 urdf model. 

```
$ roslaunch kinematics gazebo.launch 
$ rosrun motion_demo motion_demo_node 
```