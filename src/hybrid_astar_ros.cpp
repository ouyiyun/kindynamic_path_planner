#include "kindynamic_path_planner/hybrid_astar_ros.h"

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

PLUGINLIB_EXPORT_CLASS(kindynamic_path_planner::KindynamicPlannerROS, nav_core::BaseLocalPlanner);

namespace kindynamic_path_planner {
KindynamicPlannerROS::KindynamicPlannerROS() : initialized_(false), odom_helper_("odom") {}

KindynamicPlannerROS::~KindynamicPlannerROS() {}

void KindynamicPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        //! copy adress of costmap and Transform Listener (handed over from move_base)
        costmap_ros_ = costmap_ros;
        tf_ = tf;

        //! create Node Handle with name of plugin (as used in move_base for loading)
        ros::NodeHandle private_nh("~/" + name);

        //! advertise topics (adapted global plan and predicted local trajectory)
        global_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
        local_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
        pose_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

        // make sure to update the costmap we'll use for this cycle
        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        world_model_ = new base_local_planner::CostmapModel(*costmap);
        footprint_spec_ = costmap_ros_->getRobotFootprint();

        dp_ = boost::shared_ptr<HybridAstar>(new HybridAstar(name, *world_model_, costmap, footprint_spec_));

        if (private_nh.getParam("odom_topic", odom_topic_)) {
            odom_helper_.setOdomTopic(odom_topic_);
        }

        private_nh.param<double>("xy_goal_tolerance", xy_goal_tolerance_, 0.5);
        pp_ = boost::shared_ptr<PurePursuit>(new PurePursuit(name));

        initialized_ = true;
    } else {
        ROS_WARN("This planner has already been initialized, doing nothing.");
    }
}

//! TODO:
bool KindynamicPlannerROS::isGoalReached() {
    if (!initialized_) {
        ROS_ERROR("Planner must be initialized before isGoalReached is called!");
        return false;
    }

    geometry_msgs::PoseStamped robotPose;
    if (!costmap_ros_->getRobotPose(robotPose)) {
        ROS_ERROR("Could not get robot pose!");
        return false;
    }

    geometry_msgs::PoseStamped goal = global_plan_.back();

    double dist = base_local_planner::getGoalPositionDistance(robotPose, goal.pose.position.x, goal.pose.position.y);

    double yawDiff = base_local_planner::getGoalOrientationAngleDifference(robotPose, tf::getYaw(goal.pose.orientation));

    if (dist < xy_goal_tolerance_) {
        ROS_INFO("Goal Reached!");
        return true;
    }

    //！ 以pure persuit的反馈作为goal reach 条件
    //！ 以topic 的方式实现回调

    return false;
}

//! TODO:
bool KindynamicPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& globalPlan) {
    if (!initialized_) {
        ROS_ERROR("Planner must be initialized before setPlan is called!");
        return false;
    }

    //! reset the global plan
    global_plan_.clear();
    global_plan_ = globalPlan;

    return true;
}

bool KindynamicPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd) {
    //! check if plugin initialize
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }

    //! 获取robot目前的位置
    geometry_msgs::PoseStamped global_pose;
    if (!costmap_ros_->getRobotPose(global_pose)) {
        ROS_ERROR("Could not get robot pose");
        return false;
    }

    //! update costmap
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    dp_->updateCostmap(costmap_ros_->getCostmap());

    ros::Time t1 = ros::Time::now();
    std::vector<geometry_msgs::Pose2D> path2D;
    if (!dp_->solve(global_pose, global_plan_.back(), path2D)) {
        ROS_ERROR("Kindynamic path planner can't solve!");
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        return false;
    }
    ROS_INFO("kindymic path's plan time: %lf ms", (ros::Time::now() - t1).toSec() * 1000);

    //! update path
    if (path2D.empty()) {
        ROS_ERROR("No path found!");
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        return false;
    }
    pp_->setPath(path2D);
    pp_->computeVelocityCommands(global_pose, cmd);

    nav_msgs::Path local_path;
    local_path.header.frame_id = costmap_ros_->getGlobalFrameID();
    local_path.header.stamp = ros::Time::now();
    //! path2d to path
    for (auto it = path2D.begin(); it != path2D.begin() + path2D.size(); ++it) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = costmap_ros_->getGlobalFrameID();
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = (*it).x;
        pose.pose.position.y = (*it).y;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw((*it).theta);

        local_path.poses.push_back(pose);
    }
    local_plan_pub_.publish(local_path);

    return true;
}
} // namespace kindynamic_path_planner