#ifndef KINDYNAMIC_PATH_PLANNER_HYBRID_ASTAR_H_
#define KINDYNAMIC_PATH_PLANNER_HYBRID_ASTAR_H_

#include "kindynamic_path_planner/Node2D.h"
#include "kindynamic_path_planner/Node3D.h"

#include <boost/heap/binomial_heap.hpp>

#include <base_local_planner/world_model.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <kindynamic_path_planner/dubins.h>

//! OMPL
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

namespace kindynamic_path_planner {
typedef ompl::base::SE2StateSpace::StateType State;

struct CompareNodes {
    /// Sorting 3D nodes by increasing C value - the total estimated cost
    bool operator()(const Node3D* lhs, const Node3D* rhs) const { return lhs->getC() > rhs->getC(); }
    /// Sorting 2D nodes by increasing C value - the total estimated cost
    bool operator()(const Node2D* lhs, const Node2D* rhs) const { return lhs->getC() > rhs->getC(); }
};
class HybridAstar {
public:
    HybridAstar(std::string name, base_local_planner::WorldModel& world_model, costmap_2d::Costmap2D* costmap,
                std::vector<geometry_msgs::Point> footprint_spec);
    ~HybridAstar();

    enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3 };

    int search3d(Node3D& nStart, Node3D& nGoal, Node3D* nodes3D, Node2D* nodes2D, std::vector<geometry_msgs::Pose2D>& path);

    bool solve(const geometry_msgs::PoseStamped start, const geometry_msgs::PoseStamped goal, std::vector<geometry_msgs::Pose2D>& path);

    double search2d(Node2D& start, Node2D& goal, Node2D* nodes2d);

    //! call before search3d
    void updateCostmap(costmap_2d::Costmap2D* costmap) { costmap_ = costmap; }

    //! H cost
    double calHeuristics(Node3D nPred, Node3D goal, Node2D* nodes2D);

    double movecost(const Node2D& pred, const Node2D& nSucc);

    //! collisionCheck
    bool collisionCheck(const Node2D& node);
    bool collisionCheck(const Node3D& node);

    bool nodeToIndex(const Node3D& node, int& index);

    double calActualCost(const Node3D& nPred, const Node3D& nSucc);

    std::vector<geometry_msgs::Pose2D> retrivePath(const Node3D& node);

    bool dubinsShot(const Node3D& start, const Node3D& goal, std::vector<geometry_msgs::Pose2D>& dubins_path);

    void publishTrajectorty(const std::vector<geometry_msgs::Pose2D>& traj);

private:
    costmap_2d::Costmap2D* costmap_;
    double resolution_;
    double tolerance_{10.0};
    double horizon_;
    int width_, height_;

    base_local_planner::WorldModel& world_model_; ///< @brief The world model that the controller uses for collision detection
    std::vector<geometry_msgs::Point> footprint_spec_;

    ros::Publisher kindymic_path_pub_;

    //! smooth params
    double voronoi_field_weight_;
    double obstacles_weight_;
    double curvature_weight_;
    double smooth_weight_;

    //! radius
    double inscribed_radius_;
    double circumscribed_radius_;
};
} // namespace kindynamic_path_planner

#endif
