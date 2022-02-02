#ifndef _H_RRT__
#define _H_RRT__

#include <ros/ros.h>
#include <motion_planning/RRTPath.h>
#include <kinematics/IK.h>
#include <random>
#include "motion_planning/collision_detection.h"
#include <chrono>
#include "motion_planning/node.h"
#include "motion_planning/kdtree.h"
#include "motion_planning/timing.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <sensor_msgs/JointState.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>


class RRT {

  ros::NodeHandle node_handle_;
  ros::ServiceServer rrt_service_;
  ros::ServiceClient ik_client_;

  const double step_size_;
  const float goal_bias_;
  const size_t max_iterations_;
  const size_t max_reduction_steps_;

  std::default_random_engine generator_;
  std::uniform_real_distribution<double> goal_distribution_;
  std::vector<std::uniform_real_distribution<double>> joint_distributions_;

  CollisionDetection collision_detection_;
  std::vector<std::shared_ptr<Node>> tree_;
  KdTree kdtree_;

  public:
    RRT();
    bool rrtServiceCallback(motion_planning::RRTPathRequest &req, motion_planning::RRTPathResponse &res);
  
  private:
    void initRandomJointDistribution();
    bool planRRTPath(const std::vector<double> start_position, const std::vector<double> goal_position, motion_planning::RRTPathResponse &res);
    void getRandomJointPositions(std::vector<double> &joint_positions);
    inline double getGoalChance() {
      return goal_distribution_(generator_);
    }
    void addNode(std::shared_ptr<Node> node);
    std::shared_ptr<Node> getClosestNode(std::shared_ptr<Node> node);
    void createTrajectory(motion_planning::RRTPathResponse &res, std::shared_ptr<Node> &node);
    void reducePath(motion_planning::RRTPathResponse &res);
    bool interpolate(std::vector<trajectory_msgs::JointTrajectoryPoint> &new_path, const std::vector<double> &point_a, const std::vector<double> &point_b, const size_t index_a, const size_t index_b);
    double distance(const std::vector<double> &a, const std::vector<double> &b);
};

#endif