#include "move_handler.h"

move_handler::move_handler(moveit::planning_interface::MoveGroup *_group) :
  group(_group),
  received(false),
  allHoles(false),
  running(false)
{
  group->setGoalPositionTolerance(0.00000001);
  group->setEndEffectorLink(END_EFFECTOR);
}

//This Function moves the Robot to a specific goal
bool move_handler::MoveToPose(geometry_msgs::Pose pose)
{
  geometry_msgs::PoseStamped p, pout;
  p.header.frame_id = CAMERA_VIEW_FRAME;
  p.pose = pose;

  tf::TransformListener listener;

  listener.waitForTransform(CAMERA_VIEW_FRAME, ROBOT_FRAME, ros::Time(0), ros::Duration(1.0));

  listener.transformPose(ROBOT_FRAME, p, pout);

  //ROS_INFO_STREAM(p);
  //ROS_INFO_STREAM(pout);
  ROS_INFO("Moving to pose: x[%f], y[%f], z[%f].", pout.pose.position.x, pout.pose.position.y, pout.pose.position.z);

  group->setPoseTarget(pout);
  moveit_msgs::MoveItErrorCodes state = group->move();

  if (state.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Move successful");

    sleep(2.0);
    return true;
  }
  return false;
}

void move_handler::onHolesMessageReceived(const geometry_msgs::PoseArray posesMsg)
{
  if (!received && posesMsg.poses.size() != 0) {
    ROS_INFO("Received goal with [%i] location(s).", (int)posesMsg.poses.size());
    pathArray = posesMsg;
    received = true;
  }
}


//This Function just return a Pose message

geometry_msgs::Pose move_handler::get_Pose(double pX, double pY, double pZ, double oX, double oY, double oZ, double oW)
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
geometry_msgs::Pose move_handler::get_Home_Pose()
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

void move_handler::processMove()
{
  //Checking if a path was received and if the Robot isn't in activity
  if (received &&!running) {
    running = true;

    // Moving to each goal pose
    for (int i = 0; i < pathArray.poses.size(); i++) {
      if (MoveToPose(pathArray.poses[i]))
        ROS_INFO_STREAM("Bolt number: " << i + 1 << " inserted.");
      if (i == pathArray.poses.size() - 1) {
        ROS_INFO_STREAM("All " << i + 1 << " holes closed !");
        allHoles = true;
      }
    }
  }
    //Checking if All Bolts were inserted
    if (allHoles) {
      if (MoveToPose(get_Home_Pose()))
        pathArray.poses.clear();
      running = false;
      allHoles = false;
      received = false;

    }
}
