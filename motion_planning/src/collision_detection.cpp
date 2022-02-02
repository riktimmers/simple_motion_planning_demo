#include "motion_planning/collision_detection.h"

//! Constructor, initializes the robot model loader, planning scene monitor and set the robot model
CollisionDetection::CollisionDetection() :
  robot_model_loader_("robot_description"),
  planning_scene_monitor_(new planning_scene_monitor::PlanningSceneMonitor("robot_description")) {
  kinematic_model_ = robot_model_loader_.getModel();
}

//! Function to get the latest planning scene
void CollisionDetection::resetCollisionDetection() {
  planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");
  planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor_);
  planning_scene->getCurrentStateNonConst().update();
  scene_ = planning_scene->diff();
  scene_->decoupleParent();
}

//! Function to check if current joint positions are colliding 
/**
 * \param joint_positions, current joint positions of the arm to check collision for
 * \return bool, true if joint positions are colliding, else false
 */
bool CollisionDetection::isColliding(const std::vector<double> &joint_positions) {

  collision_detection::AllowedCollisionMatrix acm = scene_->getAllowedCollisionMatrix();
  collision_detection::CollisionResult::ContactMap::const_iterator it2;
  for (it2 = collision_result_.contacts.begin(); it2 != collision_result_.contacts.end(); ++it2) {
    acm.setEntry(it2->first.first, it2->first.second, true);
  }

  robot_state::RobotState &current_state = scene_->getCurrentStateNonConst();
  current_state.setJointGroupPositions("arm", joint_positions);
  scene_->setCurrentState(current_state);
  collision_result_.clear();
  scene_->checkCollision(collision_request_, collision_result_, current_state);
  return collision_result_.collision;
}