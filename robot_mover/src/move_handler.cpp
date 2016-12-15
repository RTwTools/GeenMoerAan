#include "move_handler.h"

move_handler::move_handler(moveit::planning_interface::MoveGroup *_group) :
  group(_group)
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

  ROS_INFO("Moving to pose: x[%f], y[%f], z[%f].", pout.pose.position.x, pout.pose.position.y, pout.pose.position.z);

  group->setPoseTarget(pout);

  if (group->move().val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Move successful");

    sleep(2.0);
    return true;
  }
  return false;
}

void move_handler::onHolesMessageReceived(const geometry_msgs::PoseArray posesMsg)
{
  MoveToPose(getHomePose());
  if (posesMsg.poses.size() != 0)
  {
    ROS_INFO("Received goal with [%i] location(s).", (int) posesMsg.poses.size());
    pathArray = posesMsg;

    // Moving to each goal pose
    for (int i = 0; i < pathArray.poses.size(); i++)
    {
      if (MoveToPose(pathArray.poses[i]))
        ROS_INFO_STREAM("Bolt number: " << i + 1 << " inserted.");
    }
    ROS_INFO_STREAM("All " << pathArray.poses.size() << " holes closed !");
  }
}

//Return pose made from points

geometry_msgs::Pose move_handler::getPose(double pX, double pY, double pZ, double oX, double oY, double oZ, double oW)
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

// Returns 0 0 0 pose

geometry_msgs::Pose move_handler::getHomePose()
{
  return getPose(0.314, -0.120, 0.630, 0, 0, 0, 1);
}
