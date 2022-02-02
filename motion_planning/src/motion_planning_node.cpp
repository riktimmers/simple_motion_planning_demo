#include <ros/ros.h>
#include "motion_planning/rrt.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "motion_planning_node");
  RRT rrt;
  ros::spin();
}