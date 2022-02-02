#ifndef _H_COLLISION_DETECTION__
#define _H_COLLISION_DETECTION__

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

class CollisionDetection {

  robot_model_loader::RobotModelLoader robot_model_loader_;
  robot_model::RobotModelPtr kinematic_model_;
  collision_detection::CollisionRequest collision_request_;
  collision_detection::CollisionResult collision_result_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_scene::PlanningScenePtr scene_;

  public:
    CollisionDetection();
    bool isColliding(const std::vector<double> &joint_positions);
    void resetCollisionDetection();

};

#endif