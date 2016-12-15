#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#define PUBLISHER_TOPIC "holes"
#define NODE_NAME  "path_publisher"

#define CAMERA_VIEW_FRAME "camera_view_link"


ros::Publisher publisher;
geometry_msgs::Pose get_Pose(double pX, double pY, double pZ, double oX, double oY, double oZ, double oW);

int main(int argc, char *argv[]) {

    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    geometry_msgs::PoseStamped pose1, pose2, pose3;
    nav_msgs::Path pathArray;

    pose1.header.frame_id = CAMERA_VIEW_FRAME;
    pose2.header.frame_id = CAMERA_VIEW_FRAME;
    pose3.header.frame_id = CAMERA_VIEW_FRAME;

    //Poses gotten from transform: camera_view_link --> tool0
    pose1.pose = get_Pose(0.1, 0.15, 0.005, 0.707, -0.000, -0.707, 0.000); //1
    pose2.pose = get_Pose(0.15, 0.2, 0.005, 0.707, -0.000, -0.707, 0.000); //2
    pose3.pose = get_Pose(0.2, 0.1, 0.005, 0.707, -0.000, -0.707, 0.000); //3

    pathArray.poses.push_back(pose1);
    pathArray.poses.push_back(pose2);
    pathArray.poses.push_back(pose3);
    pathArray.header.frame_id = CAMERA_VIEW_FRAME;

    publisher = n.advertise<nav_msgs::Path>(PUBLISHER_TOPIC, 1);
    ros::Rate rate(1);

    while (ros::ok()) {
        publisher.publish(pathArray);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

geometry_msgs::Pose get_Pose(double pX, double pY, double pZ, double oX, double oY, double oZ, double oW) {
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

