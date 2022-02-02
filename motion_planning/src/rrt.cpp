#include "motion_planning/rrt.h"

//! Constructor
/** Constructor initializes some global variables and initialized the service and connects to the IK service
 */
RRT::RRT() :
    step_size_(0.1), // Step size (scalar) for extending the tree
    max_iterations_(50000), // Max iterations until path is not found
    goal_bias_(0.05), // Probability of selecting the goal as the new node
    goal_distribution_(0.0, 1.0), // Distribution for selecting the goal
    max_reduction_steps_(100), // Max steps for smoothing the path
    kdtree_(max_iterations_), // Initialize the KdTree to reserve a capacity of max_iterations_
    generator_(std::chrono::system_clock::now().time_since_epoch().count()) // Give genenerator a random seed
  {
  initRandomJointDistribution();
  rrt_service_ = node_handle_.advertiseService("plan_rrt", &RRT::rrtServiceCallback, this); 
  ik_client_ = node_handle_.serviceClient<kinematics::IK>("/inverse_kinematics");
}

//! Function that sets the min and max joint values for the Kuka KR 210 
void RRT::initRandomJointDistribution() {
  joint_distributions_.reserve(6);
  joint_distributions_.push_back(std::uniform_real_distribution<double>(-185.0 * M_PI / 180.0, 185.0 * M_PI / 180.0));
  joint_distributions_.push_back(std::uniform_real_distribution<double>(-45.0 * M_PI / 180.0, 85.0 * M_PI / 180.0));
  joint_distributions_.push_back(std::uniform_real_distribution<double>(-210.0 * M_PI / 180.0, 65.0 * M_PI / 180.0));
  joint_distributions_.push_back(std::uniform_real_distribution<double>(-350.0 * M_PI / 180, 350.0 * M_PI / 180.0));
  joint_distributions_.push_back(std::uniform_real_distribution<double>(-120.0 * M_PI / 180.0, 120 * M_PI / 180.0));
  joint_distributions_.push_back(std::uniform_real_distribution<double>(-350.0 * M_PI / 180.0, 350.0 * M_PI / 180.0));
}

//! Fuction service callback to start planning and returning the found trajectory
/** 
 * \param req, contains the goal pose 
 * \param res, will contain the found trajectory
 * \return bool, true if a path was found, false if no path was found
 */
bool RRT::rrtServiceCallback(motion_planning::RRTPathRequest &req, motion_planning::RRTPathResponse &res) {
  kinematics::IKRequest ik_request;
  kinematics::IKResponse ik_response;

  std::vector<double> start_position;
  std::vector<double> goal_position;

  // Grab the current joint positions
  sensor_msgs::JointStateConstPtr current_joint_positions;
  current_joint_positions = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", node_handle_);

  if (current_joint_positions == nullptr) {
    ROS_WARN("Could not get joint_states");
    return false;
  }

  start_position = current_joint_positions->position;
  ik_request.pose = req.goal_pose;

  // Get the joint positions of the goal pose 
  if (!ik_client_.call(ik_request, ik_response)) {
    ROS_WARN("Goal pose is not valid, no IK found");
    return false; 
  }

  // Select the closest goal joint positions to the current starting joint positions
  if (std::abs(ik_response.joint_state.at(0).position.at(3) - start_position.at(3)) < std::abs(ik_response.joint_state.at(1).position.at(3) - start_position.at(3))) {
    goal_position = ik_response.joint_state.at(0).position;
  } else {
    goal_position = ik_response.joint_state.at(1).position;
  }

  // Start planning 
  return planRRTPath(start_position, goal_position, res);
}

//! Function to set the joint positions to a random position based on their min and max values
void RRT::getRandomJointPositions(std::vector<double> &joint_positions) {
  joint_positions.reserve(joint_distributions_.size());

  for (size_t index = 0; index < joint_distributions_.size(); ++index) {
    joint_positions.push_back(joint_distributions_.at(index)(generator_));
  }
}

//! Function to add a node, if it is not colliding, find the closest node and extend it into the node's direction
void RRT::addNode(std::shared_ptr<Node> node) {

  // If the given joint positions are colliding, return 
  if (collision_detection_.isColliding(node->joint_positions)) {
    return;
  }

  // Find the closest node in the tree 
  std::shared_ptr<Node> closest_node = getClosestNode(node);
  double distance_closest_node = closest_node->distance(node->joint_positions);
  Eigen::VectorXd unit_vector(6);

  for (size_t index = 0; index < node->joint_positions.size(); ++index) {
    unit_vector[index] =  node->joint_positions.at(index) - closest_node->joint_positions.at(index);
  }
  unit_vector.normalize(); // Convert the vector from closest node to node to a unit vector

  std::shared_ptr<Node> new_node(new Node(closest_node->joint_positions));

  // Take a step from the closest node into the direction of node, using step size
  for (size_t index = 0; index < new_node->joint_positions.size(); ++index) {
    new_node->joint_positions.at(index) += unit_vector[index] * step_size_;
  }

  // Check again if this new extended node is colliding, if so return
  if (collision_detection_.isColliding(new_node->joint_positions)) {
    return;
  }

  // Assign the closest node as its parent
  new_node->parent_node = closest_node;
  tree_.push_back(new_node);
  kdtree_.insertNode(new_node);
}

//! Function to get the closest node from the KdTree
/**
 * \param node, the node for which to find the closest node in the tree
 * \return std::shared_ptr<Node>, that is the closest node that is found
 */
std::shared_ptr<Node> RRT::getClosestNode(std::shared_ptr<Node> node) {
  return kdtree_.findNode(node);
}

//! Function to perform the RRT Path planning
/**
 * \param start_position, the start position in joint positions
 * \param goal_position, the goal position in joint positions
 * \param res, the response message to append the path to
 * \return bool, returns true if a path was found, otherwise false
 */
bool RRT::planRRTPath(const std::vector<double> start_position, const std::vector<double> goal_position, motion_planning::RRTPathResponse &res) {
  tree_.clear(); // reset the tree
  kdtree_.clear(); // reset the KdTree
  collision_detection_.resetCollisionDetection(); // Reset the collision detection
  tree_.reserve(max_iterations_); // Reserse space in the vector

  std::shared_ptr<Node> start_node(new Node(start_position)); // Set the starting node
  tree_.push_back(start_node);
  kdtree_.insertNode(start_node);

  // Iteration for max_iterations_ to try and find a path
  for (size_t iteration = 0; iteration < max_iterations_; ++iteration) {
    std::shared_ptr<Node> new_node(new Node());  

    // Chance to select the goal as the new node
    if (getGoalChance() <= goal_bias_) { 
      new_node->joint_positions = goal_position;
    } else {
      getRandomJointPositions(new_node->joint_positions);
    }

    // Add the node 
    addNode(new_node);

    // If the newly added node is close enough to the goal, we have found a path
    if (tree_.at(tree_.size() - 1)->distance(goal_position) < step_size_) {
      std::shared_ptr<Node> goal_node(new Node(goal_position));
      std::shared_ptr<Node> closest_node = getClosestNode(goal_node);
      goal_node->parent_node = closest_node;
      tree_.push_back(goal_node);
      kdtree_.insertNode(goal_node);
      createTrajectory(res, goal_node); // Extract the path and convert to a trajectory
      ROS_INFO_STREAM("Goal found in " << iteration << " iterations");
      return true;
    }
  }
  
  return false;
}

//! Function to create the trajectory
/**
 * \param res, the response message to contain the trajectory
 * \param node, the goal node 
 */
void RRT::createTrajectory(motion_planning::RRTPathResponse &res, std::shared_ptr<Node> &node) {

  // Extract the nodes by reversing to the start using the parent_node
  while (node->parent_node != nullptr) {
    trajectory_msgs::JointTrajectoryPoint waypoint;
    waypoint.positions = node->joint_positions;
    res.trajectory.points.push_back(waypoint);
    node = node->parent_node;
  }

  trajectory_msgs::JointTrajectoryPoint waypoint;
  waypoint.positions = tree_.at(0)->joint_positions;
  res.trajectory.points.push_back(waypoint);

  // Reverse the path since it went from goal -> start
  std::reverse(res.trajectory.points.begin(), res.trajectory.points.end());
  reducePath(res); // Smooth the path

  // TEMP, adding time parameterization here
  // @TODO Implement own time parameterization
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  robot_trajectory::RobotTrajectory robot_trajectory(robot_model, "arm");
  trajectory_processing::IterativeParabolicTimeParameterization iptp(100, 0.01);
  robot_state::RobotState state(robot_model);

  for (size_t index = 0; index < res.trajectory.points.size(); ++index) {
    state.setJointGroupPositions("arm", res.trajectory.points.at(index).positions);
    robot_trajectory.addSuffixWayPoint(state, 0.0);
  }

  iptp.computeTimeStamps(robot_trajectory, 1.0, 1.0);

  for (size_t index = 0; index < robot_trajectory.getWayPointCount(); ++index) {
    res.trajectory.points.at(index).positions.clear();
    
    for (size_t j = 0; j < 6; ++j) {
      res.trajectory.points.at(index).positions.push_back(robot_trajectory.getWayPoint(index).getVariablePosition(j));
      res.trajectory.points.at(index).velocities.push_back(robot_trajectory.getWayPoint(index).getVariableVelocity(j));
      res.trajectory.points.at(index).accelerations.push_back(robot_trajectory.getWayPoint(index).getVariableAcceleration(j));
    }
    res.trajectory.points.at(index).time_from_start = ros::Duration(robot_trajectory.getWayPointDurationFromStart(index));
  }
}

//! Function to smooth the path by reducing the amount of points and find shortcuts 
void RRT::reducePath(motion_planning::RRTPathResponse &res) {

  // For max_reductions_steps, try to perform reduction of the path 
  for (size_t index = 0; index < max_reduction_steps_; ++index) {
    // Select two random points
    std::uniform_int_distribution<size_t> index_distribution(0, res.trajectory.points.size() - 1);
    size_t index_a = index_distribution(generator_);
    size_t index_b = index_distribution(generator_);

    if (index_a == index_b) { // Points need to be different
      continue;
    }

    if (index_a > index_b) { // index_a needs to be smaller then index_b
      std::swap(index_a, index_b);
    }

    const std::vector<double> point_a = res.trajectory.points.at(index_a).positions;
    const std::vector<double> point_b = res.trajectory.points.at(index_b).positions;

    double distance_a_direct_b = distance(point_a, point_b);
    double distance_a_sum_b = 0.0;

    for (size_t point_index = index_a; point_index < index_b; ++point_index) {
      distance_a_sum_b += distance(res.trajectory.points.at(point_index).positions, res.trajectory.points.at(point_index + 1).positions);
    }

    // If the direct path is shorter, try and reduce it 
    if (distance_a_direct_b <= distance_a_sum_b) {
      std::vector<trajectory_msgs::JointTrajectoryPoint> new_path;

      // If the linear interpolation between the 2 points exists add that path and remove the other points
      if (interpolate(new_path, point_a, point_b, index_a, index_b)) {
        res.trajectory.points.erase(res.trajectory.points.begin()+index_a, res.trajectory.points.begin()+index_b);
        res.trajectory.points.insert(res.trajectory.points.begin()+index_a, new_path.begin(), new_path.end());
      }
    }
  }

  // remove points with to small distance
  std::vector<size_t> remove_indices;
  for (size_t index = 0; index < res.trajectory.points.size() - 1; ++index) {
    if (distance(res.trajectory.points.at(index).positions, res.trajectory.points.at(index+1).positions) < 0.01) {
      remove_indices.push_back(index);
    }
  }

  for (auto index: remove_indices) {
    res.trajectory.points.erase(res.trajectory.points.begin()+index);
  }
}

//! Function to linearly interpolate between two joint positions
/**
 * \param new_path, will contain the new path between the joint positions
 * \param point_a, the first joint positions
 * \param point_b, the second joint positions
 * \param index_a, the index of point_a in the total trajectory
 * \param index_b, the index of point_b in the total trajectory
 */
bool RRT::interpolate(std::vector<trajectory_msgs::JointTrajectoryPoint> &new_path, const std::vector<double> &point_a, const std::vector<double> &point_b, const size_t index_a, const size_t index_b) {
  const size_t steps = index_b - index_a;
  const float step_size = 1.0 / steps;

  trajectory_msgs::JointTrajectoryPoint interpolate_point;
  interpolate_point.positions.resize(point_a.size());
  
  // interpolate between 0 - 1, bases on the number of steps
  for (float t = 0; t < 1.0; t += step_size) {

    for (size_t index = 0; index < point_a.size(); ++index) {
      interpolate_point.positions.at(index) = point_a.at(index) * (1.0 - t)  + point_b.at(index) * t;
    }

    // if an interpolated point collides, return false
    if (collision_detection_.isColliding(interpolate_point.positions)) {
      return false;
    }

    // Add interpolated path to new path, if it has a distance larger or equal to step_size_
    if (new_path.size() && distance(new_path.back().positions, interpolate_point.positions) >= step_size_) {
      new_path.push_back(interpolate_point);
    } else { // If there is no points in the new_path, add the first point
      new_path.push_back(interpolate_point);
    }   
  }

  // If no points are added at least add the first point
  if (new_path.size() == 0) {
    interpolate_point.positions = point_a;
    new_path.push_back(interpolate_point);
  }

  return true;
}

//! Function that returns the distance (Euclidean) between joint positions
/**
 * \param a, joint positions a
 * \param b, joint positions b
 * \return double, the Euclidian distance between the two points
 */
double RRT::distance(const std::vector<double> &a, const std::vector<double> &b) {
  double sum = 0;

  for (size_t index = 0; index < a.size(); ++index) {
    sum += std::pow(a.at(index) - b.at(index), 2);
  }

  return std::sqrt(sum);
}