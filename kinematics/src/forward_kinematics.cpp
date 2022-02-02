#include "kinematics/kinematics.h"

//! Function callback for the published joint_states, in order to calculate the forward kinematics
void Kinematics::jointStateCallback(const sensor_msgs::JointStatePtr &joint_states) {
  std::vector<double> joint_positions;
  joint_positions.reserve(6);

  for (size_t index = 0; index < 6; ++index) {
    joint_positions.push_back(joint_states->position.at(index));
  }

  calculateForwardKinematics(joint_positions, tcp_);
  tcp_publisher_.publish(tcp_);
}

//! Function to calculate the forward kinematics
/**
 * \param joint_positions, are the current joint positions
 * \param tcp, the message for containing the TCP position
 */
void Kinematics::calculateForwardKinematics(const std::vector<double> &joint_positions, kinematics::TCP &tcp) {
  Eigen::Matrix3d R1 = getZRotationMatrix(joint_positions.at(0));
  Eigen::Matrix3d R2 = getYRotationMatrix(joint_positions.at(1));
  Eigen::Matrix3d R3 = getYRotationMatrix(joint_positions.at(2));
  Eigen::Matrix3d R4 = getXRotationMatrix(joint_positions.at(3));
  Eigen::Matrix3d R5 = getYRotationMatrix(joint_positions.at(4));
  Eigen::Matrix3d R6 = getXRotationMatrix(joint_positions.at(5));

  // Hardcoded transformations (@TODO make variables out of this)
  Eigen::Matrix4d T1 = getTransformationMatrix(R1, 0, 0, 0.33);
  Eigen::Matrix4d T2 = getTransformationMatrix(R2, 0.35, 0, 0.42);
  Eigen::Matrix4d T3 = getTransformationMatrix(R3, 0, 0, 1.25);
  Eigen::Matrix4d T4 = getTransformationMatrix(R4, 0.96, 0, -0.054);
  Eigen::Matrix4d T5 = getTransformationMatrix(R5, 0.54, 0, 0);
  Eigen::Matrix4d T6 = getTransformationMatrix(R6, 0.230, 0, 0);

  Eigen::Matrix3d rotation = R1*R2*R3*R4*R5*R6;
  Eigen::Matrix4d translation = T1*T2*T3*T4*T5*T6;

  double r31 = rotation(2, 0);
  double r33 = rotation(2, 2);
  double r32 = rotation(2, 1);
  double r11 = rotation(0, 0);
  double r21 = rotation(1, 0);

  tcp.x = translation(0, 3);
  tcp.y = translation(1, 3);
  tcp.z = translation(2, 3);
  tcp.pitch = std::atan2(-r31, std::sqrt(std::pow(r11,2) + std::pow(r21, 2))); 
  tcp.roll = std::atan2(r32 / std::cos(tcp.pitch), r33 / std::cos(tcp.pitch));
  tcp.yaw = std::atan2(r21 / std::cos(tcp.pitch), r11 / std::cos(tcp.pitch));
}

//! Function that returns the Z Rotation Matrix
/** 
 * \param theta, the joint rotation
 * \return Eigen::Matrix3d, the Rotation Matrix around the Z axis
 */
Eigen::Matrix3d Kinematics::getZRotationMatrix(double theta) {
  Eigen::Matrix3d z_rotation;
  z_rotation << std::cos(theta), -std::sin(theta), 0,
                std::sin(theta),  std::cos(theta), 0,
                              0,                0, 1;
  return z_rotation;
}

//! Function that returns the Y Rotation Matrix
/** 
 * \param theta, the joint rotation
 * \return Eigen::Matrix3d, the Rotation Matrix around the Y axis
 */
Eigen::Matrix3d Kinematics::getYRotationMatrix(double theta) {
  Eigen::Matrix3d y_rotation;
  y_rotation << std::cos(theta), 0, std::sin(theta),
                              0, 1,               0,
               -std::sin(theta), 0, std::cos(theta);
  return y_rotation;
}

//! Function that returns the X Rotation Matrix
/** 
 * \param theta, the joint rotation
 * \return Eigen::Matrix3d, the Rotation Matrix around the X axis
 */
Eigen::Matrix3d Kinematics::getXRotationMatrix(double theta) {
  Eigen::Matrix3d x_rotation;
  x_rotation << 1,               0,                0,
                0, std::cos(theta), -std::sin(theta),
                0, std::sin(theta),  std::cos(theta);
  return x_rotation;
}

//! Function that returns the Transormation Matrix
/** 
 * \param rotation_matrix, the Rotation Matrix for the Translation
 * \param x, the x translation
 * \param y, the y translation
 * \param z, the z translation
 * \return Eigen::Matrix4d, returns the Translation matrix
 */
Eigen::Matrix4d Kinematics::getTransformationMatrix(Eigen::Matrix3d rotation_matrix, double x, double y, double z) {
  Eigen::Matrix4d transformation_matrix;
  transformation_matrix.block<3, 3>(0, 0) = rotation_matrix;
  transformation_matrix(0, 3) = x;
  transformation_matrix(1, 3) = y;
  transformation_matrix(2, 3) = z;
  transformation_matrix(3, 3) = 1;
  return transformation_matrix;
}