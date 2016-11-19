#include "ros/ros.h"
#include "MoveHandler.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/PoseArray.h>
#define SUBSCRIBER_TOPIC "holes"
#define NODE_NAME  "Path_Publisher"


ros::Publisher publisher;
geometry_msgs::Pose get_Pose(double pX, double pY, double pZ, double oX, double oY, double oZ, double oW);

int main(int argc, char *argv[])
{
	/* code */
ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle n;

  geometry_msgs::Pose pose1, pose2,pose3 ,pose4;
  geometry_msgs::PoseArray pathArray;

//Poses gotten from transform: my_camera_view_link --> tool0
  pose1= get_Pose(0.214, 0.240, 0.089, 0.70711, 0.146, -0.70711, -0.403);//1
  
 pose2= get_Pose(0.280, 0.336, 0.050, 0.70711, 0.399, -0.70711, -0.469); //2
    
  pose3  =get_Pose(0.317, 0.183, 0.115, 0.70711, 0.146, -0.70711, -0.403);//Move back To Home
  pose4= get_Pose(0.405, 0.170, 0.162,  0.70711, 0.000,-0.70711, 1.000);//4
        
  pathArray.poses.push_back(pose1);
  pathArray.poses.push_back(pose2);
  pathArray.poses.push_back(pose3);

 pathArray.poses.push_back(pose4);

  publisher = n.advertise<geometry_msgs::PoseArray>(SUBSCRIBER_TOPIC, 1000);

  sleep(1.0);
  publisher.publish(pathArray);
 
  ros::spinOnce();
	return 0;
}




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
