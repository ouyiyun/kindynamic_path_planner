#include "kindynamic_path_planner/Node3D.h"

namespace kindynamic_path_planner {
Node3D* Node3D::createSuccessor(const int i) {
    double xSucc, ySucc, tSucc;

    int m = common::dir / 2;

    int id = i % m;

    // successor in parent node coordinate
    int forward = i < m ? 1 : -1;
    int reflectT = i < m ? 0 : 1;
    double x = forward * common::dx[id];
    double y = common::dy[id];
    double t = theta_; // theta

    // successor in map coordinate
    xSucc = x * cos(t) - y * sin(t) + coord_x_;
    ySucc = x * sin(t) + y * cos(t) + coord_y_;
    tSucc = common::mod2pi(t + forward * common::dt[id] + M_PI * reflectT);

    return new Node3D(xSucc, ySucc, tSucc, g_, 0, this, i);
}

// get trajectory from this node's parent
std::vector<geometry_msgs::Pose2D> Node3D::getTrajectory() const {
    std::vector<geometry_msgs::Pose2D> poses;
    poses.clear();

    // nullptr
    if (!pred_) return poses;
    // trajectory size
    int len = sizeof(common::traX[0]) / sizeof(common::traX[0][0]);

    int m = common::dir / 2;
    int id = prim_ % m;

    int forward = prim_ < m ? 1 : -1;
    int reflectT = prim_ < m ? 0 : 1;

    // parent pose
    double t = pred_->getTheta();
    double a = pred_->getX();
    double b = pred_->getY();

    for (int i = 0; i < len; ++i) {
        geometry_msgs::Pose2D pose;

        double x = forward * common::traX[id][i];
        double y = common::traY[id][i];

        pose.x = x * cos(t) - y * sin(t) + a;
        pose.y = x * sin(t) + y * cos(t) + b;
        pose.theta = common::mod2pi(t + forward * common::traT[id][i] + reflectT * M_PI);

        poses.push_back(pose);
    }

    return poses;
}
} // namespace kindynamic_path_planner