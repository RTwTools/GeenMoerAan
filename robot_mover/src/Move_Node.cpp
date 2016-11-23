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
#define CAMERA_VIEW_FRAME "my_camera_view_link"



//Transform done between my_camera_view_link --> tool0
using namespace std;

bool MoveToPose(moveit::planning_interface::MoveGroup *group, geometry_msgs::Pose pose);
geometry_msgs::Pose get_Pose(double pX, double pY, double pZ, double oX, double oY, double oZ, double oW);
void holes_callBack(geometry_msgs::PoseArray posesMsg);
geometry_msgs::Pose get_Home_Pose();

char nodeName [] = NODE_NAME;
bool received = false;
bool allHoles=false;
bool running=false;
geometry_msgs::PoseArray pathArray;


void holes_callBack(geometry_msgs::PoseArray posesMsg)
{
    ROS_INFO_STREAM("Path received !!");
    pathArray = posesMsg;
    received=true;
}
int main(int argc, char *argv[])
{
  ros::init(argc, argv, nodeName);
  ros::NodeHandle n;
  moveit::planning_interface::MoveGroup group(GROUPID);
  ros::Subscriber subHoles = n.subscribe(SUBSCRIBER_TOPIC, 1, &holes_callBack);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(ros::ok())
  {   
      //Checking if a path was received and if the Robot isn't in activity
      if(received &&!running)
      { running=true;
      
            // Moving to each goal pose
            for(int i=0; i<pathArray.poses.size(); i++)
            {
                if (MoveToPose(&group, pathArray.poses[i]))
                ROS_INFO_STREAM("  bolt Nr: " <<i+1<<" inserted !!!!!");
                if(i==pathArray.poses.size()-1) {ROS_INFO_STREAM("All "<<i+1<< " holes closed !!!!!"); allHoles=true; received =false;}
            }
      }
      //Checking if All Bolts were inserted 
      if(allHoles)
      {
          if (MoveToPose(&group,get_Home_Pose()))
             ROS_INFO_STREAM(" *** Returned to Home ***");
          running= false;
          allHoles=false;
      }
  }
  ros::waitForShutdown();
  return 0;
}

//This Function moves the Robot to a specific goal

bool MoveToPose(moveit::planning_interface::MoveGroup *group, geometry_msgs::Pose pose)
{
 move_group_interface::MoveGroup::Plan plan;
 group->setGoalPositionTolerance(0.00000001);
 group->setPoseTarget(pose,END_EFFECTOR);
 group->setPoseReferenceFrame(CAMERA_VIEW_FRAME);
  moveit_msgs::MoveItErrorCodes success = group->plan(plan);
  
    if(success.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_INFO("Planning successful! Now move");
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

// This function generates The Home position pose

geometry_msgs::Pose get_Home_Pose()
{
  geometry_msgs::Pose homePose;
  homePose.position.x = 0.314;
  homePose.position.y = -0.120;
  homePose.position.z = 0.630;

  homePose.orientation.x = -0.000;
  homePose.orientation.y = -0.000;
  homePose.orientation.z = 0.000;
  homePose.orientation.w = 1.000;
  return homePose;
}
