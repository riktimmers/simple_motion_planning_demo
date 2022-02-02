#include <ros/ros.h>
#include "kinematics/kinematics.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "kinematics_node");

  Kinematics kinematics;

  ros::spin();
}