#ifndef _H_KINEMATICS__
#define _H_KINEMATICS__

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <kinematics/TCP.h>
#include <kinematics/IK.h>
#include <sensor_msgs/JointState.h>
#include <tf2/utils.h>

class Kinematics {

  ros::Subscriber joint_state_subscriber_;
  ros::Publisher tcp_publisher_;
  ros::NodeHandle node_handle_;

  kinematics::TCP tcp_;
  ros::ServiceServer inverse_kinematics_server_;

  public:
    Kinematics();
    void jointStateCallback(const sensor_msgs::JointStatePtr &joint_states);
  
  private:
    void calculateForwardKinematics(const std::vector<double> &joint_positions, kinematics::TCP &tcp_);
    Eigen::Matrix3d getZRotationMatrix(double theta);
    Eigen::Matrix3d getYRotationMatrix(double theta);
    Eigen::Matrix3d getXRotationMatrix(double theta);
    Eigen::Matrix4d getTransformationMatrix(Eigen::Matrix3d rotation_matrix, double x, double y, double z);
    bool ikServiceCallback(kinematics::IKRequest &req, kinematics::IKResponse &res);
};

#endif