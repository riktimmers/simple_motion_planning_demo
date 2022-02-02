#include "motion_demo/motion_demo.h"

//! Constructor 
/** Constructor uses the joint_controller to set the robot's position at 0 for all joints.
 *  It connects to the plan_rrt service, initializes all the goals and starts running the demo.
 */
MotionDemo::MotionDemo() {
  
  joint_controller_.setZeroPosition();
  rrt_path_client_ = node_handle_.serviceClient<motion_planning::RRTPath>("/plan_rrt");
  rrt_path_client_.waitForExistence();

  initGoals();
  runDemo();
}

//! Function to set the goals for the demo, the Pose are x,y,z coordinates and roll, pitch, yaw in radians for the end effector link.
void MotionDemo::initGoals() {
  goals_.emplace_back(Pose(2.08, 0.0, 0.5, 0, 0, 0));
  goals_.emplace_back(Pose(-2.0, 0.5, 1.5, 0, 0, M_PI));
  goals_.emplace_back(Pose(1.5, 0, 1.5, 0, M_PI_2, 0));
  goals_.emplace_back(Pose(1.5, 1.4, 1.5, 0, 0, M_PI_2));
  goals_.emplace_back(Pose(0.2, -1.6, 0.9, M_PI/3.0, -M_PI_4, -M_PI_2));
  goals_.emplace_back(Pose(2.08, 0.0, 1.945, 0, 0, 0));
}

//! Function runs the demo, by looping through all the waypoints once 
void MotionDemo::runDemo() {

  for (size_t index = 0; index < goals_.size(); ++index) {
    tf2::Quaternion quaternion;
    geometry_msgs::Pose goal_pose; 
    goal_pose.position.x = goals_.at(index).x;
    goal_pose.position.y = goals_.at(index).y;
    goal_pose.position.z = goals_.at(index).z;
    quaternion.setRPY(goals_.at(index).roll, goals_.at(index).pitch, goals_.at(index).yaw);
    goal_pose.orientation = tf2::toMsg(quaternion); // Convert RPY Euler angles to quaternion

    motion_planning::RRTPathRequest request;
    request.goal_pose = goal_pose;
    motion_planning::RRTPathResponse response;

    // Request to plan with RRT a trajectory to the given goal pose 
    if (!rrt_path_client_.call(request, response)) {
      ROS_WARN("Could not find path");
      return;
    }

    // Publish the Goal TF (eef_goal) for visualization in RViz
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "eef_goal";
    transformStamped.transform.translation.x = goals_.at(index).x;
    transformStamped.transform.translation.y = goals_.at(index).y;
    transformStamped.transform.translation.z = goals_.at(index).z;
    transformStamped.transform.rotation.x = quaternion.x();
    transformStamped.transform.rotation.y = quaternion.y();
    transformStamped.transform.rotation.z = quaternion.z();
    transformStamped.transform.rotation.w = quaternion.w();

    tf_broadcaster_.sendTransform(transformStamped);

    executeTrajectory(response.trajectory); // Execute the trajectory
  }
}

//! Function that executes the trajectory
/**
 * \param trajectory is a trajectory_msgs::JointTrajectory that contains all the variables 
 * position, velocity and acceleration for each waypoint.
 */
void MotionDemo::executeTrajectory(trajectory_msgs::JointTrajectory &trajectory) {

  for (size_t index = 0; index < trajectory.points.size()-1; ++index) {
    trajectory_msgs::JointTrajectoryPoint point0 = trajectory.points.at(index);
    trajectory_msgs::JointTrajectoryPoint point1 = trajectory.points.at(index+1);

    const double total_time = point1.time_from_start.toSec() - point0.time_from_start.toSec();
    joint_controller_.setJointPosition(point0.positions, point0.velocities, point0.accelerations,
                                       point1.positions, point1.velocities, point1.accelerations,
                                       total_time);
  } 
}
