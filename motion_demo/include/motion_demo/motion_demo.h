#ifndef _H_MOTION_DEMO__
#define _H_MOTION_DEMO__

#include <ros/ros.h>
#include <joint_controller/joint_controller.h>
#include <motion_planning/RRTPath.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

struct Pose {
  double x;
  double y;
  double z; 
  double roll; 
  double pitch;
  double yaw;

  Pose(double _x, double _y, double _z, 
       double _roll, double _pitch, double _yaw) :
       x(_x), y(_y), z(_z), 
       roll(_roll), pitch(_pitch), yaw(_yaw) {};
};

class MotionDemo {

  JointController joint_controller_;
  ros::ServiceClient rrt_path_client_;
  ros::NodeHandle node_handle_;
  std::vector<Pose> goals_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  public:
    MotionDemo();
    void runDemo();
  
  private:
    void initGoals();
    void executeTrajectory(trajectory_msgs::JointTrajectory &trajectory);
};
#endif