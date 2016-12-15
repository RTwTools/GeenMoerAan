#include "move_handler.h"

move_handler::move_handler(moveit::planning_interface::MoveGroup *_group) :
group(_group) {
    group->setGoalTolerance(0.005);
    group->setGoalPositionTolerance(0.00000001);
    group->setEndEffectorLink(END_EFFECTOR);

}

//This Function moves the Robot to a specific goal

bool move_handler::MoveToPose(geometry_msgs::Pose pose) {

    ROS_INFO("Moving to pose: x[%f], y[%f], z[%f].", pose.position.x, pose.position.y, pose.position.z);

    group->setPoseTarget(pose);

    if (group->move().val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_INFO("Move successful");

        sleep(2.0);
        return true;
    }
    return false;
}

void move_handler::onHolesMessageReceived(const nav_msgs::Path pathMsg) {
    goHome();
    if (pathMsg.poses.size() != 0) {
        ROS_INFO("Received goal with [%i] location(s).", (int) pathMsg.poses.size());
        pathArray = pathMsg;

        // Moving to each goal pose
        for (int i = 0; i < pathArray.poses.size(); i++) {
            geometry_msgs::PoseStamped p = pathArray.poses[i];
            p.pose.position.z += 0.075;
            if (MoveToPose(translatePose(p).pose))
                if (MoveToPose(translatePose(pathArray.poses[i]).pose))
                    if (MoveToPose(translatePose(p).pose))
                        ROS_INFO_STREAM("Bolt number: " << i + 1 << " inserted.");
        }
        ROS_INFO_STREAM("All " << pathArray.poses.size() << " holes closed !");
        goHome();
    }
}

//Return pose made from points

geometry_msgs::Pose move_handler::getPose(double pX, double pY, double pZ, double oX, double oY, double oZ, double oW) {
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

void move_handler::goHome() {
    MoveToPose(getPose(0.22, 0.29, 0.60, -0.1047, -0.0367, 0.4414, 0.8904));
}

geometry_msgs::PoseStamped move_handler::translatePose(geometry_msgs::PoseStamped poseStamped) {
    geometry_msgs::PoseStamped pout;

    tf::TransformListener listener;

    listener.waitForTransform(poseStamped.header.frame_id, ROBOT_FRAME, ros::Time(0), ros::Duration(1.0));
    listener.transformPose(ROBOT_FRAME, poseStamped, pout);
    return pout;
}