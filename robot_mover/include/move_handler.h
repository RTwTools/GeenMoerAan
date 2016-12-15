#ifndef MOVEHANDLER_H
#define MOVEHANDLER_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

#define SUBSCRIBER_TOPIC          "holes"
#define GROUPID                   "manipulator"
#define SUBSCRIBER_BUFFER_SIZE    1
#define END_EFFECTOR              "tool0"
#define NODE_NAME                 "Move_Handler_Node"
#define ROBOT_FRAME               "base_link"

using namespace std;

class move_handler {
private:
    moveit::planning_interface::MoveGroup *group;
    nav_msgs::Path pathArray;

public:
    move_handler(moveit::planning_interface::MoveGroup *_group);
    bool MoveToPose(geometry_msgs::Pose pose);
    geometry_msgs::Pose getPose(double pX, double pY, double pZ, double oX, double oY, double oZ, double oW);
    void goHome();
    void onHolesMessageReceived(nav_msgs::Path posesMsg);
    geometry_msgs::PoseStamped translatePose(geometry_msgs::PoseStamped poseStamped);
};
#endif
