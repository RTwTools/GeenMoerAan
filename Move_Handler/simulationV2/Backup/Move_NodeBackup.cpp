#include "ros/ros.h"
#include "MoveHandler.h"
#include "tf/transform_listener.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"




ros::Publisher *planning_scene_diff_publisher  = new ros::Publisher();
tf::TransformListener *listener;

int main(int argc, char *argv[])
{
	/* code */
ros::init(argc, argv, "nodeName");

  ros::NodeHandle n;
listener=new tf::TransformListener();
ros::AsyncSpinner spinner(1);
   if(spinner.canStart())spinner.start();
 moveit::planning_interface::MoveGroup group("manipulator");
  *planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1000);
  MoveHandler *obj= new MoveHandler(planning_scene_diff_publisher,&group,listener);
  ros::Subscriber sub = n.subscribe("holes", 1000, &MoveHandler::path_CallBack,obj);
   sleep(3.0);
  
    

 // obj->current_Pose_CallBack();
   geometry_msgs::Pose pose1, pose2,pose3 ,pose4;
   pose1.position.x =0.299;
   pose1.position.y =-0.229;
   pose1.position.z =0.487;
   pose1.orientation.x=-0.000;
   pose1.orientation.y=-0.000;
   pose1.orientation.z=0.000;
   pose1.orientation.w=1.000;
   
   
   pose2.position.x =0.314;
   pose2.position.y =-0.120;
   pose2.position.z =0.630;
   pose2.orientation.x=-0.000;
   pose2.orientation.y=-0.000;
   pose2.orientation.z=0.000;
   pose2.orientation.w=1.000;
   
   
   pose3.position.x =0.289;
   pose3.position.y =-0.152;
   pose3.position.z =0.090;
   pose3.orientation.x=-0.000;
   pose3.orientation.y=0.000;
   pose3.orientation.z=0.881;
   pose3.orientation.w=0.472;
   
   std::vector<geometry_msgs::Pose> data;
	 data.push_back(pose2);
         data.push_back(pose1);
         data.push_back(pose3);



   obj->Move(data);
  ros::spin();
	return 0;
}





