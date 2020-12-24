#ifndef KINDYNAMIC_PATH_PLANNER_HYBRID_ASTAR_ROS_H_
#define KINDYNAMIC_PATH_PLANNER_HYBRID_ASTAR_ROS_H_

#include "kindynamic_path_planner/hybrid_astar.h"
#include "kindynamic_path_planner/pure_pursuit.h"

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf2_ros/buffer.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <base_local_planner/costmap_model.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>

namespace kindynamic_path_planner {
class KindynamicPlannerROS : public nav_core::BaseLocalPlanner {
public:
    /**
     * @brief Constructor
     */
    KindynamicPlannerROS();

    /**
     * @brief Destructor
     */
    ~KindynamicPlannerROS();

    /**
     * @brief  Constructs the ros wrapper
     * @param name The name to give this instance of the trajectory planner
     * @param tf A pointer to a transform listener
     * @param costmap The cost map to use for assigning costs to trajectories
     */
    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief Sets the global plan to be followed by this planner
     * @param globalPlan: The plan to follow
     * @return true if plan was set successfully
     */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& globalPlan);

    /**
     * @brief Computes the velocity command for the robot base
     * @param cmd: The computed command container
     * @return true if a cmd was computed successfully
     */
    bool computeVelocityCommands(geometry_msgs::Twist& cmd);

    /**
     * @brief Checks whether the goal is reached
     * @return true if goal is reached
     */
    bool isGoalReached();

private:
    void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

    void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

    //! global plan publisher
    ros::Publisher global_plan_pub_;
    //! local plan publisher
    ros::Publisher local_plan_pub_;
    ros::Publisher pose_pub_;

    //! global plan
    std::vector<geometry_msgs::PoseStamped> global_plan_;

    costmap_2d::Costmap2DROS* costmap_ros_; //! pointer to costmap
    tf2_ros::Buffer* tf_;                   //! pointer to Transform Listener

    bool initialized_;

    base_local_planner::OdometryHelperRos odom_helper_;
    std::string odom_topic_;

    boost::shared_ptr<HybridAstar> dp_;
    base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use
    std::vector<geometry_msgs::Point> footprint_spec_;

    //! pure pursuit
    double xy_goal_tolerance_;

    boost::shared_ptr<PurePursuit> pp_;
};
} // namespace kindynamic_path_planner
#endif