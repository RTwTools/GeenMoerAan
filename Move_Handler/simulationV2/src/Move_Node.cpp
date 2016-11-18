//This file is made with the objective to simulate the usage of Rviz
#include "ros/ros.h"
#include "moveit/move_group_interface/move_group.h"
#include  <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <time.h>
#include <string.h>

#define SUBSCRIBER_TOPIC "holes"
#define GROUPID "manipulator"
#define SUBSCRIBER_BUFFER_SIZE 1000
#define END_EFFECTOR "tool0"
#define NODE_NAME "Move_Handler_Node"


//Transform done between my_camera_view_link --> tool0
using namespace std;

bool MoveToPose(moveit::planning_interface::MoveGroup *group, geometry_msgs::Pose pose);
geometry_msgs::Pose get_Pose(double pX, double pY, double pZ, double oX, double oY, double oZ, double oW);
void holes_callBack(geometry_msgs::PoseArray posesMsg);

char nodeName [] = NODE_NAME;
bool received = false;
geometry_msgs::PoseArray pathArray;

void holes_callBack(geometry_msgs::PoseArray posesMsg)
{
    ROS_INFO_STREAM("Path received !!");
    pathArray = posesMsg;
    received=true;
    ROS_INFO_STREAM(pathArray);
}
int main(int argc, char *argv[])
{
  ros::init(argc, argv, nodeName);
  ros::NodeHandle n;
  moveit::planning_interface::MoveGroup group(GROUPID);
  ros::Subscriber subHoles = n.subscribe(SUBSCRIBER_TOPIC, 1000, &holes_callBack);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(ros::ok())
  {
      if(received)
      {
            
            for(int i=0; i<pathArray.poses.size(); i++)
            {
                if (MoveToPose(&group, pathArray.poses[i]))
                ROS_INFO_STREAM("  bolt inserted !!!!!");
                if(i==pathArray.poses.size()-1) {ROS_INFO_STREAM("All holes closed !!!!!");received =false;}
            }
      }
  }
  ros::waitForShutdown();
  return 0;
}

//This Function moves the Robot to a specific goal

bool MoveToPose(moveit::planning_interface::MoveGroup *group, geometry_msgs::Pose pose)
{
 move_group_interface::MoveGroup::Plan plan;
 
 group->setPoseTarget(pose,END_EFFECTOR);
  moveit_msgs::MoveItErrorCodes success = group->plan(plan);
    if(success.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_INFO("Success! Now move");
        success = group->move();
        sleep(5.0);       
        return true;
    }
  return false;
}

//This Function just return a Pose message

geometry_msgs::Pose get_Pose(double pX, double pY, double pZ, double oX, double oY, double oZ, double oW)
{
  geometry_msgs::Pose pose;
  pose.position.x = pX;
  pose.position.y = pY;
  pose.position.z = pZ;

  pose.orientation.x = oX;
  pose.orientation.y = oY;
  pose.orientation.z = oZ;
  pose.orientation.w = oW;
  return pose;
}