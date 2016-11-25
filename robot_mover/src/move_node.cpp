#include "ros/ros.h"
#include "move_handler.h"
#include "tf/transform_listener.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"


int main(int argc, char *argv[])
{
  ros::init(argc, argv, NODE_NAME);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle n;  
  moveit::planning_interface::MoveGroup group(GROUPID);
  move_handler *obj= new move_handler(&group);
  ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC,SUBSCRIBER_BUFFER_SIZE, &move_handler::onHolesMessageReceived,obj);
  
  while(ros::ok()){

    obj->processMove();

  }
  ros::waitForShutdown();

  return 0;
}




