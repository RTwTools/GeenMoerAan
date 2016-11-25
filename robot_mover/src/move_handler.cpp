#include "move_handler.h"

//moveit::planning_interface::MoveGroup temp("manipulator");

move_handler::move_handler(ros::Publisher *_publisher, moveit::planning_interface::MoveGroup *_group, tf::TransformListener *_listener)

: group(_group),
publisher(_publisher),
listener(_listener) {
}
//MoveHandler::~MoveHandler();

bool move_handler::Move(geometry_msgs::Pose pose) {

    ROS_INFO_STREAM("Entered MOVE");
    //moveit::planning_interface::MoveGroup g("manipulator");
    move_group_interface::MoveGroup::Plan plan;

    group->setPoseTarget(pose, "tool0");

    moveit_msgs::MoveItErrorCodes success = group->plan(plan);
    if (success.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_INFO("Success! Now move");
        success = group->move();
        sleep(5.0);
        return true;
    }
    return false;

}

void move_handler::set_Path(const nav_msgs::Path path) {

    Coordinates = path;
}

nav_msgs::Path move_handler::get_Path() const {

    return Coordinates;
}

void move_handler::path_CallBack(const nav_msgs::Path &path) {

    //Coordinates=path;
    tf::StampedTransform transform;
    ros::Time now = ros::Time::now();
    if (listener->waitForTransform("map", "tool0", now, ros::Duration(2.0))) {
        set_Path(path);
        ROS_INFO_STREAM("transform found ");
        listener->lookupTransform("map", "tool0", now, transform);
        double phi = 2.0 * asin(transform.getRotation().z());
        std::vector<geometry_msgs::PoseStamped> data = path.poses;
        std::vector<geometry_msgs::Pose> hole_poses;
        for (int i = 0; i < data.size(); i++) {
            double x_t = data[i].pose.position.x - transform.getOrigin().x(); //translation
            double y_t = data[i].pose.position.y - transform.getOrigin().y(); //translation
            double x_r = x_t * cos(phi) + y_t * sin(phi);
            double y_r = -x_t * sin(phi) + y_t * cos(phi);
            geometry_msgs::Pose p;
            p.position.x = x_r;
            p.position.y = y_r;
            p.position.z = 0;
            hole_poses.push_back(p);

        }
        if (!hole_poses.empty()) {
            for (int i = 0; i < hole_poses.size(); i++) {
                Move(hole_poses[i]);
            }
        }

    }

}

geometry_msgs::Pose move_handler::current_Pose_CallBack() {
    geometry_msgs::Pose tempPose;
    tf::StampedTransform transform;
    ros::Time now = ros::Time::now();
    if (listener->waitForTransform("map", "base_link", now, ros::Duration(2.0))) {
        ROS_INFO_STREAM(" found");
        listener->lookupTransform("map", "base_link", now, transform);
        group->setEndEffectorLink("tool0");

        //get the current PoseStamped
        geometry_msgs::PoseStamped current_pose = group->getCurrentPose();
        tempPose.position.x = current_pose.pose.position.x;
        tempPose.position.y = current_pose.pose.position.y;
        tempPose.position.z = current_pose.pose.position.z;
        tempPose.orientation.x = current_pose.pose.orientation.x;
        tempPose.orientation.y = current_pose.pose.orientation.y;
        tempPose.orientation.z = current_pose.pose.orientation.z;
        tempPose.orientation.w = current_pose.pose.orientation.w;

        //ROS_INFO_STREAM(tempPose);
    } else ROS_INFO_STREAM("not found");
    return tempPose;
}
// geometry_msgs::Pose MoveHandler::resetArm()const{
//  	return null;
//  }
