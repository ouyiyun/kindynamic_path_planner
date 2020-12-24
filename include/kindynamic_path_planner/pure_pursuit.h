#ifndef KINDYNAMIC_PATH_PLANNER_PURE_PURSUIT_H_
#define KINDYNAMIC_PATH_PLANNER_PURE_PURSUIT_H_

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace kindynamic_path_planner {
class PurePursuit {
public:
    PurePursuit(std::string name);
    ~PurePursuit() = default;

    void setPath(std::vector<geometry_msgs::Pose2D> path_in) { path = path_in; }

    void computeVelocityCommands(geometry_msgs::PoseStamped& robot_pose, geometry_msgs::Twist& cmd);

private:
    std::vector<geometry_msgs::Pose2D> path;
    double wheel_base_;
    double reference_vel_;
    double forward_distance_;
    double max_angle_limit_, min_angle_limit_;
};
} // namespace kindynamic_path_planner
#endif