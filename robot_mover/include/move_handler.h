#ifndef MOVEHANDLER_H
#define MOVEHANDLER_H
#include "ros/ros.h"
#include "moveit/move_group_interface/move_group.h"
#include  <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <time.h>
#include <string.h>

#define SUBSCRIBER_TOPIC          "holes"
#define GROUPID                   "manipulator"
#define SUBSCRIBER_BUFFER_SIZE    1
#define END_EFFECTOR              "tool0"
#define NODE_NAME                 "Move_Handler_Node"
#define CAMERA_VIEW_FRAME         "camera_view_link"
#define ROBOT_FRAME               "base_link"

using namespace std;

class move_handler
{

private: 
  moveit::planning_interface::MoveGroup *group;
  geometry_msgs::PoseArray pathArray;



public:
  move_handler(moveit::planning_interface::MoveGroup *_group );
  bool MoveToPose( geometry_msgs::Pose pose);
  geometry_msgs::Pose getPose(double pX, double pY, double pZ, double oX, double oY, double oZ, double oW);
  geometry_msgs::Pose getHomePose();
  void onHolesMessageReceived(geometry_msgs::PoseArray posesMsg);

};
#endif
