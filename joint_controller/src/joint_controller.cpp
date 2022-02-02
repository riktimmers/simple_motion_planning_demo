#include "joint_controller/joint_controller.h"

//! Constructor.
/** Constructor creates joint publishers for each of the joints
 */
JointController::JointController() {
  joint_publishers_.reserve(6);
  
  for (size_t index = 1; index < 7; ++index) {
    joint_publishers_.push_back(node_handle_.advertise<std_msgs::Float64>("/kuka_arm/joint"+std::to_string(index)+"_controller/command", 1));
  }
}

//! Function to interpolate between two positions 
/**
 * \param q0, is the starting position
 * \param qt, is the goal position
 * \param qd0, is the starting velocity
 * \param qdt, is the goal velocity
 * \param qdd0, is the starting acceleration
 * \param qddt, is the goal acceleration
 * \param T, is the total time of the interpolation
 * \param t, is the current time of the interpolation
 */
double JointController::interpolate(const double q0, const double qt, const double qd0, 
                                    const double qdt, const double qdd0, const double qddt,
                                    const double T, const double t) {
  Eigen::MatrixXd mat(6, 6);
  mat << 0, 0, 0, 0, 0, 1,
         std::pow(T, 5), std::pow(T ,4), std::pow(T, 3), std::pow(T, 2), T, 1, 
         0, 0, 0, 0, 1, 0, 
         5*std::pow(T, 4), 4*std::pow(T, 3), 3*std::pow(T,2), 2*T, 1, 0, 
         0, 0, 0, 2, 0, 0,
         20*std::pow(T,3), 12*std::pow(T,2), 6*T, 2, 0, 0;
  Eigen::VectorXd vec(6);
  vec << q0, qt, qd0, qdt, qdd0, qddt;
  Eigen::VectorXd coeef = mat.inverse()*vec;

  double A = coeef[0];
  double B = coeef[1];
  double C = coeef[2];
  double D = coeef[3];
  double E = coeef[4];
  double F = coeef[5];
  
  return A*std::pow(t, 5) + B*std::pow(t, 4) + C*std::pow(t, 3) + D*std::pow(t, 2) + E*t + F;
}

//! Function to set the joint position of the arm, using interpolation
/**
 * \param start_positions, the starting joint positions
 * \param start_velocities, the starting joint velocities
 * \param start_accelerations, the starting accelerations
 * \param final_positions, the goal joint positions 
 * \param final_velocities, the goal joint velocities
 * \param final_accelerations, the goal accelerations
 * \param total_time, the total time for the interpolation
 */ 
void JointController::setJointPosition(const std::vector<double> &start_positions, 
                                       const std::vector<double> &start_velocities,
                                       const std::vector<double> &start_accelerations,
                                       const std::vector<double> &final_positions,
                                       const std::vector<double> &final_velocities, 
                                       const std::vector<double> &final_accelerations,
                                       const double total_time) {
  
  const size_t hz = 1000; // Control each joint at 1kHz
  ros::Rate rate(hz);
  double time_steps = 1.0 / hz; // Time step in seconds
  std::vector<double> position(6, 0); // For getting the current position in the interpolation
    
  for (double t = 0.0; t < total_time; t += time_steps) {
    for (size_t index = 0; index < start_positions.size(); ++index) {
      position.at(index) = interpolate(start_positions.at(index), final_positions.at(index),
                                  start_velocities.at(index), final_velocities.at(index),
                                  start_accelerations.at(index), final_accelerations.at(index),
                                  total_time, t);
    }

    // Publish each new joint position 
    for (size_t index = 0; index < position.size(); ++index) {
      joint_publishers_.at(index).publish(position.at(index));
    }

    rate.sleep();
    ros::spinOnce();
  }
}

//! Function to set the Kuka arm to have all 0 position values
void JointController::setZeroPosition() {
  
  ros::Rate rate(5);
  for (size_t i = 0; i < 10; ++i) {  // Make sure it executes, this is just for init stuff
    for (size_t index = 0; index < joint_publishers_.size(); ++index) {
      joint_publishers_.at(index).publish(0.0);
    }
    ros::spinOnce();
    rate.sleep();
  }
}