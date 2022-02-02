#include "kinematics/kinematics.h"

//! Constructor
/** Constructor initializes the forward kinematics publisher, subscribes to the joint_states and advertises the inverse kinematics server
 */
Kinematics::Kinematics() {
  tcp_publisher_ = node_handle_.advertise<kinematics::TCP>("/tcp", 0);  
  joint_state_subscriber_ = node_handle_.subscribe("/joint_states", 0, &Kinematics::jointStateCallback, this); 
  inverse_kinematics_server_ = node_handle_.advertiseService("/inverse_kinematics", &Kinematics::ikServiceCallback, this);
}

//! Function to calculate the inverse kinematics (@TODO might not return all possible solution!)
/** 
 * \param req, the request for the end effector pose from which to calculate the inverse kinematics
 * \param res, the response that will contain the solutions (joint positions)
 */
bool Kinematics::ikServiceCallback(kinematics::IKRequest &req, kinematics::IKResponse &res) {
  const double x = req.pose.position.x;
  const double y = req.pose.position.y;
  const double z = req.pose.position.z;
  tf2::Quaternion quaternion;
  tf2::fromMsg(req.pose.orientation, quaternion);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quaternion).getEulerYPR(yaw, pitch, roll); // Get the Roll, Pitch, Yaw Euler angles from the Quaternion

  const Eigen::Matrix3d rotation_x = getXRotationMatrix(roll); // Get the X Rotation Matrix
  const Eigen::Matrix3d rotation_y = getYRotationMatrix(pitch); // Get the Y Rotation Matrix
  const Eigen::Matrix3d rotation_z = getZRotationMatrix(yaw); // Get the Z Rotation Matrix
  const Eigen::Matrix3d rotation = rotation_z * rotation_y * rotation_x; 
  const Eigen::Vector3d x_hat = rotation * Eigen::Vector3d::UnitX();
  
  Eigen::Vector3d WP;
  const Eigen::Vector3d TCP(x, y, z);

  WP = TCP - 0.230 * x_hat;

  const double WPxy = std::sqrt(std::pow(WP.x(), 2) + std::pow(WP.y(), 2));
  const double l = WPxy - 0.35; 
  const double h  = WP.z() - 0.33 - 0.42; 
  const double rho = std::sqrt(std::pow(l, 2) + std::pow(h, 2));
  const double a3z = 1.25;
  const double b4x = std::sqrt(std::pow(1.5, 2) + std::pow(0.054, 2));

  if (rho > a3z + b4x || rho < std::abs(a3z - b4x)) {
    return false;
  }

  const double alpha = std::atan2(h, l);
  const double beta = std::acos((std::pow(rho,2) + std::pow(a3z, 2) - std::pow(b4x, 2)) / (2*rho*a3z)); 
  const double theta2 = M_PI_2 - alpha - beta; 

  const double gamma = std::acos((std::pow(a3z, 2) + std::pow(b4x, 2) - std::pow(rho, 2)) / (2*a3z*b4x));
  const double delta = std::atan2(b4x, 0.054); 

  const double theta3 = -gamma + delta; 
  const double theta1 = std::atan2(WP.y(), WP.x()); 

  const Eigen::Matrix3d rotation_theta1 = getZRotationMatrix(theta1);
  const Eigen::Matrix3d rotation_theta2 = getYRotationMatrix(theta2);
  const Eigen::Matrix3d rotation_theta3 = getYRotationMatrix(theta3);

  const Eigen::Matrix3d rotation_arm = rotation_theta1 * rotation_theta2 * rotation_theta3;

  const Eigen::Matrix3d rotation_wrist = rotation_arm.transpose() * rotation;
  const double r11 = rotation_wrist(0, 0);
  const double r21 = rotation_wrist(1, 0);
  const double r31 = rotation_wrist(2, 0);
  const double r12 = rotation_wrist(0, 1);
  const double r13 = rotation_wrist(0, 2);
  const double r32 = rotation_wrist(2, 1);
  const double r33 = rotation_wrist(2, 2);

  double theta5 = std::atan2(std::sqrt(1-std::pow(r11,2)), r11); 
  double theta4 = std::atan2(r21, -r31); 
  double theta6 = std::atan2(r12, r13); 
  
  // If theta 5 == 0.0 (or very close to it), pick theta4 as the current joint position to calculate theta6
  if (theta5 < 0.0 + std::numeric_limits<double>::epsilon() & theta5 > 0.0 - std::numeric_limits<double>::epsilon()) {
    sensor_msgs::JointStateConstPtr current_joint_positions;
    current_joint_positions = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", node_handle_);

    if (current_joint_positions == nullptr) {
      ROS_WARN("Could not get joint_states");
      return false;
    }

    theta4 = current_joint_positions->position.at(3);
    theta6 = std::atan2(r32, r33) - theta4; 
  }

  sensor_msgs::JointState joint_state; 
  joint_state.position.push_back(theta1);
  joint_state.position.push_back(theta2);
  joint_state.position.push_back(theta3);
  joint_state.position.push_back(theta4);
  joint_state.position.push_back(theta5);
  joint_state.position.push_back(theta6);  

  res.joint_state.push_back(joint_state);

  // Second solution for the wrist joints 
  theta5 = std::atan2(-std::sqrt(1-std::pow(r11,2)), r11);
  theta4 = std::atan2(-r21, r31); 
  theta6 = std::atan2(-r12, -r13); 

  joint_state.position.at(3) = theta4;
  joint_state.position.at(4) = theta5;
  joint_state.position.at(5) = theta6;

  res.joint_state.push_back(joint_state);

  return true;
}