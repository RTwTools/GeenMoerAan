#include "ros/ros.h"
#include "moveit/move_group_interface/move_group.h"
#include  <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include "nav_msgs/Path.h"
#include "tf/transform_listener.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"


 class move_handler
{

private: 
	moveit::planning_interface::MoveGroup *group;
	ros::Publisher *publisher;
	nav_msgs::Path Coordinates;
	tf::TransformListener *listener;



public:

	move_handler(ros::Publisher *publisher,moveit::planning_interface::MoveGroup *_group, tf::TransformListener * listener);
	//~MoveHandler();
	bool Move(geometry_msgs::Pose pose) ;
	void  set_Path(const nav_msgs::Path path);
	nav_msgs::Path get_Path() const;
	void path_CallBack(const nav_msgs::Path &path);
	geometry_msgs::Pose current_Pose_CallBack() ;
	geometry_msgs::Pose resetArm()const;




};
