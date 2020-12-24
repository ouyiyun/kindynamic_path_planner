#include "kindynamic_path_planner/Node3D.h"

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

using namespace kindynamic_path_planner;

void publishPath(ros::Publisher& pub, std::vector<geometry_msgs::Pose2D>& pts) {
    std::cout << "pts.size() = " << pts.size() << std::endl;

    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";

    for (auto it = pts.begin(); it != pts.begin() + pts.size(); ++it) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = (*it).x;
        pose.pose.position.y = (*it).y;

        pose.pose.orientation = tf::createQuaternionMsgFromYaw((*it).theta);

        path.poses.push_back(pose);
    }

    pub.publish(path);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test");

    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 100);

    Node3D* node = new Node3D(.0, .0, .0, .0, .0, nullptr, 0);

    for (int i = 0; i < common::dir / 2; ++i) {
        Node3D* succ = node->createSuccessor(i);

        std::vector<geometry_msgs::Pose2D> pts = succ->getTrajectory();
        publishPath(path_pub, pts);
        ros::Duration(0.2).sleep();
    }

    std::cout << "hellow ros\n";

    return 1;
}