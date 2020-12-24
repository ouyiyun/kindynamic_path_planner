#include "kindynamic_path_planner/pure_pursuit.h"
#include <tf/transform_datatypes.h>
#include "kindynamic_path_planner/common.h"

namespace kindynamic_path_planner {
PurePursuit::PurePursuit(std::string name) {
    ros::NodeHandle private_nh("~/" + name);

    private_nh.param<double>("wheel_base", wheel_base_, 2.95);
    private_nh.param<double>("reference_vel", reference_vel_, 1.0);
    private_nh.param<double>("forward_distance", forward_distance_, 2.0);
    private_nh.param<double>("max_angle_limit", max_angle_limit_, 2.0);
    private_nh.param<double>("min_angle_limit", min_angle_limit_, -2.0);

    path.clear();
}

void PurePursuit::computeVelocityCommands(geometry_msgs::PoseStamped& robot_pose, geometry_msgs::Twist& cmd) {
    //! current state
    double cx = robot_pose.pose.position.x;
    double cy = robot_pose.pose.position.y;
    double ct = tf::getYaw(robot_pose.pose.orientation);
    //! 搜索最临近的路点
    int size = path.size();
    int index = 0;
    double L = 0.0;
    double x_error, y_error, distance;

    while (L < forward_distance_ && (index + 1) < size) {
        x_error = path[index + 1].x - path[index].x;
        y_error = path[index + 1].y - path[index].y;
        distance = x_error * x_error + y_error * y_error;
        distance = std::sqrt(distance);
        L += distance;
        index++;
    }

    //! our goal pose
    double gx, gy;
    if (index >= size) {
        gx = path.back().x;
        gy = path.back().y;
    } else {
        gx = path[index].x;
        gy = path[index].y;
    }

    double alpha = ct - atan2(gy - cy, gx - cx);

    alpha = common::mod2pi(alpha);

    double delta = atan2(2 * wheel_base_ * sin(alpha), forward_distance_);
    ROS_INFO("delta: %lf", delta);

    if (delta > max_angle_limit_) {
        delta = max_angle_limit_;
    } else if (delta < min_angle_limit_) {
        delta = min_angle_limit_;
    }

    cmd.linear.x = reference_vel_;
    cmd.angular.z = -delta;
}
} // namespace kindynamic_path_planner