#include "motion_demo/motion_demo.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "motion_demo");  
  MotionDemo motion_demo;
  ros::spin();  
}