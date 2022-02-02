#ifndef _H_JOINT_CONTROLLER__
#define _H_JOINT_CONTROLLER__

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Eigen>

class JointController {

  std::vector<ros::Publisher> joint_publishers_;
  ros::NodeHandle node_handle_;

  public:
    JointController();
    void setZeroPosition();
    double interpolate(const double q0, const double qt, const double qd0, 
                       const double qdt, const double qdd0, const double qddt,
                       const double T, const double t);
    void setJointPosition(const std::vector<double> &start_positions, 
                          const std::vector<double> &start_velocities,
                          const std::vector<double> &start_accelerations,
                          const std::vector<double> &final_positions,
                          const std::vector<double> &final_velocities, 
                          const std::vector<double> &final_accelerations,
                          const double total_time);
};

#endif