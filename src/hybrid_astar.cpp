#include "kindynamic_path_planner/hybrid_astar.h"

#include <costmap_2d/footprint.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

namespace kindynamic_path_planner {
//! TODO:
HybridAstar::HybridAstar(std::string name, base_local_planner::WorldModel& world_model, costmap_2d::Costmap2D* costmap,
                         std::vector<geometry_msgs::Point> footprint_spec)
    : world_model_(world_model), footprint_spec_(footprint_spec) {
    ros::NodeHandle private_nh("~/" + name);
    costmap_ = costmap;
    resolution_ = costmap_->getResolution();

    kindymic_path_pub_ = private_nh.advertise<nav_msgs::Path>("kindymic_path", 1);

    private_nh.param<double>("voronoi_field_weight", voronoi_field_weight_, 1.0);
    private_nh.param<double>("obstacles_weight", obstacles_weight_, 1.0);
    private_nh.param<double>("curvature_weight", curvature_weight_, 1.0);
    private_nh.param<double>("smooth_weight", smooth_weight_, 1.0);

    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, inscribed_radius_, circumscribed_radius_);
    ROS_INFO("inscribed_radius_: %lf, circumscribed_radius_: %lf", inscribed_radius_, circumscribed_radius_);
    double map_x = costmap_->getSizeInMetersX();
    double map_y = costmap_->getSizeInMetersY();

    horizon_ = std::min(map_x / 2.0, map_y / 2.0) * 0.9;
    tolerance_ = 0.9 * horizon_;
}

//! TODO:
HybridAstar::~HybridAstar() {}

bool HybridAstar::solve(const geometry_msgs::PoseStamped start, const geometry_msgs::PoseStamped goal, std::vector<geometry_msgs::Pose2D>& path) {
    path.clear();
    //! update costmap info in this cycle
    resolution_ = costmap_->getResolution();
    width_ = costmap_->getSizeInCellsX();
    height_ = costmap_->getSizeInCellsY();

    //! define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[width_ * height_]();
    Node2D* nodes2D = new Node2D[width_ * height_]();

    double x = start.pose.position.x;
    double y = start.pose.position.y;
    double theta = tf::getYaw(start.pose.orientation);
    Node3D nStart(x, y, theta, 0, 0, nullptr);

    x = goal.pose.position.x;
    y = goal.pose.position.y;
    theta = tf::getYaw(goal.pose.orientation);
    Node3D nGoal(x, y, theta, .0, .0, nullptr);

    double map_x = costmap_->getSizeInMetersX();
    double map_y = costmap_->getSizeInMetersY();

    horizon_ = std::min(map_x / 2.0, map_y / 2.0) * 0.9;
    tolerance_ = 0.9 * horizon_;

    int state = search3d(nStart, nGoal, nodes3D, nodes2D, path);

    if (state == NO_PATH || path.empty()) {
        return false;
    }

    delete[] nodes2D;
    delete[] nodes3D;

    return true;
}

int HybridAstar::search3d(Node3D& nStart, Node3D& nGoal, Node3D* nodes3D, Node2D* nodes2D, std::vector<geometry_msgs::Pose2D>& path) {
    //! Number of possible directions, 7 for forward driving and an additional 7 for reversing
    int dir = common::reverse ? common::dir : common::dir / 2;

    //! PREDECESSOR AND SUCCESSOR INDEX
    int iPred, iSucc;
    double newG, newH;

    //! priority queue as open list
    boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>> Q;

    //! init start node
    nStart.putInOpenedList();
    newH = calHeuristics(nStart, nGoal, nodes2D);
    nStart.setH(newH);
    nStart.setG(.0);

    //! push on priority queue aka open list
    nodeToIndex(nStart, iPred);
    nodes3D[iPred] = nStart;
    Q.push(&nodes3D[iPred]);

    int iteration = 0;

    Node3D *nPred, *nSucc;

    while (!Q.empty()) {
        iteration++;

        // ROS_INFO("iteration: %d", iteration);
        nPred = Q.top();
        // std::vector<geometry_msgs::Pose2D> traj = nPred->getTrajectory();
        // unsigned int size = traj.size();
        // publishTrajectorty(traj);
        // ros::Duration(0.1).sleep();

        //! set index
        nodeToIndex(*nPred, iPred);
        nPred->setIndex(iPred);

        if (nodes3D[iPred].isInClosedList()) {
            Q.pop();
            continue;
        }

        //! add node to closed list
        nodes3D[iPred].putInClosedList();
        //! remove node from open list
        Q.pop();

        bool near_end = std::abs(nPred->getX() - nGoal.getX()) <= tolerance_ && std::abs(nPred->getY() - nGoal.getY()) <= tolerance_;

        if (near_end) {
            std::vector<geometry_msgs::Pose2D> dubins_path;
            if (dubinsShot(*nPred, nGoal, dubins_path)) {
                ROS_INFO("Reach end goal!");
                ROS_INFO("iteration: %d", iteration);
                std::vector<geometry_msgs::Pose2D> kindynamic_path = retrivePath(*nPred);
                path.insert(path.end(), kindynamic_path.begin(), kindynamic_path.begin() + kindynamic_path.size());
                path.insert(path.end(), dubins_path.begin(), dubins_path.begin() + dubins_path.size());
                return REACH_END;
            }
        }

        bool reach_horizon = std::pow(nPred->getX() - nStart.getX(), 2) + std::pow(nPred->getY() - nStart.getY(), 2) >= std::pow(horizon_, 2);
        //! reach_horizon
        if (reach_horizon) {
            std::vector<geometry_msgs::Pose2D> kindynamic_path = retrivePath(*nPred);
            path = kindynamic_path;
            ROS_INFO("Reach horizon!");
            ROS_INFO("iteration: %d", iteration);
            return REACH_HORIZON;
        }

        //! update neighbors
        for (int i = 0; i < dir; ++i) {
            nSucc = nPred->createSuccessor(i);
            // std::vector<geometry_msgs::Pose2D> traj = nSucc->getTrajectory();
            // unsigned int size = traj.size();
            // publishTrajectorty(traj);
            // ros::Duration(0.1).sleep();
            if (!nodeToIndex(*nSucc, iSucc)) {
                delete nSucc;
                continue;
            }
            nSucc->setIndex(iSucc);

            if (!nodes3D[iSucc].isInClosedList()) {
                if (collisionCheck(*nSucc)) {
                    //! calculate new G value
                    newG = calActualCost(*nPred, *nSucc);
                    nSucc->setG(newG);

                    if (!nodes3D[iSucc].isInOpenedList() || newG < nodes3D[iSucc].getG()) {
                        //! heuristics
                        newH = calHeuristics(*nSucc, nGoal, nodes2D);
                        nSucc->setH(newH);

                        //! put in open
                        nSucc->putInOpenedList();

                        //! update node
                        nodes3D[iSucc] = *nSucc;
                        Q.push(&nodes3D[iSucc]);
                        delete nSucc;
                    } else
                        delete nSucc;
                } else
                    delete nSucc;
            }
        }
    }

    return NO_PATH;
} // namespace kindynamic_path_planner

double HybridAstar::calHeuristics(Node3D nPred, Node3D goal, Node2D* nodes2D) {
    double sx = nPred.getX(), sy = nPred.getY(), gx = goal.getX(), gy = goal.getY();

    //! non-holonomic-without-obstacles
    ompl::base::ReedsSheppStateSpace reedsSheppPath(common::maxRadius);
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();
    rsStart->setXY(sx, sy);
    rsStart->setYaw(nPred.getTheta());
    rsEnd->setXY(gx, gy);
    rsEnd->setYaw(goal.getTheta());

    // reeds shepp cost
    double reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);

    // Euclidean cost
    double EuclideanCost = sqrt((sx - gx) * (sx - gx) + (sy - gy) * (sy - gy));

    // uint mx, my;
    // if (!costmap_->worldToMap(sx, sy, mx, my)) {
    //     // ROS_ERROR("start pos out of map!");
    //     return common::inf;
    // }
    // Node2D start2D = Node2D(mx, my, .0, .0, nullptr);

    // if (!costmap_->worldToMap(gx, gy, mx, my)) {
    //     // ROS_ERROR("goal pos out of map!");
    //     return common::inf;
    // }
    // Node2D goal2D = Node2D(mx, my, .0, .0, nullptr);

    // double twoDCost = search2d(start2D, goal2D, nodes2D);

    return std::max(reedsSheppCost, EuclideanCost);
    // return std::max(twoDCost, std::max(reedsSheppCost, EuclideanCost));
}

double HybridAstar::calActualCost(const Node3D& nPred, const Node3D& nSucc) {
    // reference: https://zhuanlan.zhihu.com/p/40776683
    double g = nPred.getG();

    int m = common::dir / 2;
    int nSuccPrim = nSucc.getPrim();
    int nPredPrim = nPred.getPrim();
    double s = common::chordLength[nSuccPrim % m];

    // forward
    if (nSuccPrim < m)
        g = g + s;
    else
        g = g + common::penaltyReverse * s;

    // change of direction
    if ((nPredPrim < m && nSuccPrim >= m) || (nPredPrim >= m && nSuccPrim < m)) {
        g = g + common::penaltyChangeDirection * s;
    }

    // steer angle
    g = g + common::penaltyTurnAngle * fabs(common::steerAngle[nSuccPrim % m]);

    // change steer angle
    g = g + common::penaltyChangeSteerAngle * fabs(common::steerAngle[nPredPrim % m] - common::steerAngle[nSuccPrim % m]);
    return g;
}

double HybridAstar::search2d(Node2D& start, Node2D& goal, Node2D* nodes2d) {
    // initial opne and closed list
    int iPred, iSucc;
    double newG, newH;

    for (int i = 0; i < width_ * height_; ++i) {
        nodes2d[i].reset();
    }

    //! priority queue
    std::priority_queue<Node2D*, std::vector<Node2D*>, CompareNodes> Q;
    newH = movecost(start, goal);
    start.setH(newH);
    start.setG(.0);

    //! put in open list
    start.putInOpenedList();

    //! push in priority queue
    Q.push(&start);

    //! node pointer
    Node2D *nPred, *nSucc;

    while (!Q.empty()) {
        nPred = Q.top();

        //! set idx
        nPred->setIdx(width_);
        iPred = nPred->getIdx();

        //! if there exists a pointer this node has already been expanded
        if (nodes2d[iPred].isInClosedList()) {
            Q.pop();
            continue;
        }

        //! put in closed and nodes2d
        nodes2d[iPred].putInClosedList();

        //! remove from Q
        Q.pop();

        //! reach goal?
        if (goal == *nPred) {
            return nPred->getG();
        }

        //! neighbor search
        for (int i = 0; i < Node2D::dir; ++i) {
            //! create possible successor
            nSucc = nPred->createSuccssor(i);

            //! set index of the successor
            iSucc = nSucc->setIdx(width_);

            if (collisionCheck(*nSucc) && !nodes2d[iSucc].isInClosedList()) {
                //! update g cost
                newG = nSucc->getG() + movecost(*nPred, *nSucc);
                nSucc->setG(newG);

                if (!nodes2d[iSucc].isInOpenedList() || newG < nodes2d[iSucc].getG()) {
                    //! update h cost
                    newH = movecost(*nSucc, goal);
                    nSucc->setH(newH);

                    //! put in opened list
                    nSucc->putInOpenedList();

                    //! update node
                    nodes2d[iSucc] = *nSucc;
                    Q.push(&nodes2d[iSucc]);
                    delete nSucc;
                } else
                    delete nSucc;
            } else
                delete nSucc;
        }
    }

    ROS_INFO("Can't find valid path!");
    return common::inf;
}

double HybridAstar::movecost(const Node2D& nPred, const Node2D& nSucc) {
    double dx = fabs(nPred.getX() - nSucc.getX());
    double dy = fabs(nPred.getY() - nSucc.getY());

    // Diagonal distance
    double dis = dx + dy + (sqrt(2) - 2) * std::min(dx, dy);

    // Euclidean distance
    //    double dis = sqrt(dx * dx + dy * dy);

    // Manhattan distance
    //    double dis = dx + dy;
    return dis * resolution_;
}

bool HybridAstar::collisionCheck(const Node2D& node) {
    //! out of map
    if (node.getX() >= costmap_->getSizeInCellsX() || node.getY() >= costmap_->getSizeInCellsY()) {
        return false;
    }
    char c = costmap_->getCost(node.getX(), node.getY());

    //! TODO:
    if (c != costmap_2d::FREE_SPACE) {
        return false;
    }

    return true;
}

bool HybridAstar::collisionCheck(const Node3D& node) {
    std::vector<geometry_msgs::Pose2D> path = node.getTrajectory();

    double wx, wy, theta;
    unsigned int mx, my;
    char cost;

    for (auto it = path.begin(); it != path.begin() + path.size(); ++it) {
        wx = (*it).x;
        wy = (*it).y;
        theta = (*it).theta;

        //! out of map
        if (!costmap_->worldToMap(wx, wy, mx, my)) {
            return false;
        }

        //! check the point on the trajectory for legality
        double footprint_cost = world_model_.footprintCost(wx, wy, theta, footprint_spec_, inscribed_radius_, circumscribed_radius_);
        if (footprint_cost == -1) {
            // ROS_INFO("cost: %lf", footprint_cost);
            return false;
        }
        // cost = costmap_->getCost(mx, my);
        // if (cost != costmap_2d::FREE_SPACE) {
        //     return false;
        // }
    }

    return true;
}

bool HybridAstar::nodeToIndex(const Node3D& node, int& index) {
    unsigned int mx, my;

    if (!costmap_->worldToMap(node.getX(), node.getY(), mx, my)) {
        // ROS_ERROR("Out of the map");
        return false;
    }

    index = costmap_->getIndex(mx, my);

    return true;
}

std::vector<geometry_msgs::Pose2D> HybridAstar::retrivePath(const Node3D& node) {
    std::vector<geometry_msgs::Pose2D> kindynamic_path;
    kindynamic_path.clear();

    const Node3D* nPred = &node;

    while (nPred != nullptr) {
        std::vector<geometry_msgs::Pose2D> path = nPred->getTrajectory();
        kindynamic_path.insert(kindynamic_path.begin(), path.begin(), path.begin() + path.size());
        nPred = nPred->getPred();
    }

    return kindynamic_path;
}

bool HybridAstar::dubinsShot(const Node3D& start, const Node3D& goal, std::vector<geometry_msgs::Pose2D>& dubins_path) {
    dubins_path.clear();
    //! start
    double q0[] = {start.getX(), start.getY(), start.getTheta()};
    //! goal
    double q1[] = {goal.getX(), goal.getY(), goal.getTheta()};
    //! initial the path
    DubinsPath path;
    //! calculate the path
    dubins_init(q0, q1, common::maxRadius, &path);

    double x = .0;
    double dubins_length = dubins_path_length(&path);

    unsigned int mx, my;
    char cost;
    geometry_msgs::Pose2D pose;

    while (x < dubins_length) {
        double q[3];
        dubins_path_sample(&path, x, q);
        pose.x = q[0];
        pose.y = q[1];
        pose.theta = q[2];

        // collision check

        if (!costmap_->worldToMap(pose.x, pose.y, mx, my)) {
            dubins_path.clear();
            return false;
        }

        double footprint_cost = world_model_.footprintCost(q[0], q[1], q[2], footprint_spec_, inscribed_radius_, circumscribed_radius_);

        if (footprint_cost == -1 || footprint_cost == -2) {
            return false;
        }
        // if (cost != costmap_2d::FREE_SPACE) {
        //     dubins_path.clear();
        //     return false;
        // }
        x += costmap_->getResolution();

        dubins_path.push_back(pose);
    }

    // goal state push
    // pose.x = q1[0];
    // pose.x = q1[1];
    // pose.theta = q1[2];
    // dubins_path.push_back(pose);

    return true;
}

void HybridAstar::publishTrajectorty(const std::vector<geometry_msgs::Pose2D>& traj) {
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();

    for (auto it = traj.begin(); it != traj.begin() + traj.size(); ++it) {
        geometry_msgs::PoseStamped pose;

        pose.pose.position.x = (*it).x;
        pose.pose.position.y = (*it).y;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw((*it).theta);

        path.poses.push_back(pose);
    }

    kindymic_path_pub_.publish(path);
}
} // namespace kindynamic_path_planner