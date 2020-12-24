#ifndef KINDYNAMIC_PATH_PLANNER_NODE3D_H_
#define KINDYNAMIC_PATH_PLANNER_NODE3D_H_

#include <geometry_msgs/Pose2D.h>
#include <vector>

#include "kindynamic_path_planner/common.h"

namespace kindynamic_path_planner {
class Node3D {
public:
    //! the default constructor
    Node3D() : Node3D(.0, .0, .0, .0, .0, nullptr, 0) {}
    Node3D(double coord_x, double coord_y, double theta, double g, double h, const Node3D* pred, int prim = 0) {
        coord_x_ = coord_x;
        coord_y_ = coord_y;
        theta_ = theta;
        g_ = g;
        h_ = h;
        pred_ = pred;
        prim_ = prim;
        opened_ = false;
        closed_ = false;
        index_ = -1;
    }

    double getX() const { return coord_x_; }
    void setX(double coord_x) { coord_x_ = coord_x; }

    double getY() const { return coord_y_; }
    void setY(double coord_y) { coord_y_ = coord_y; }

    double getTheta() const { return theta_; }
    void setTheta(double theta) { theta_ = theta; }

    double getG() const { return g_; }
    void setG(double g) { g_ = g; }

    double getH() const { return h_; }
    void setH(double h) { h_ = h; }

    double getC() const { return g_ + h_; }

    int getIndex() const { return index_; }
    void setIndex(int index) { index_ = index; }

    bool isInOpenedList() const { return opened_; }
    void putInOpenedList() {
        opened_ = true;
        closed_ = false;
    }

    bool isInClosedList() const { return closed_; }
    void putInClosedList() {
        closed_ = true;
        opened_ = false;
    }

    int getPrim() const { return prim_; }
    void setPrim(int prim) { prim_ = prim; }

    const Node3D* getPred() const { return pred_; }
    void setPred(Node3D* pred) { pred_ = pred; }

    // generater neighbor node(Successor node)
    Node3D* createSuccessor(const int prim);

    // get trajectory for colliosion check
    std::vector<geometry_msgs::Pose2D> getTrajectory() const;

private:
    //! the x y position and heading theta(t)
    double coord_x_, coord_y_, theta_;

    // the g cost
    double g_;

    // the h cost
    double h_;

    // the index of the node in 3D array
    int index_;

    // the motion primitive of the node
    int prim_;

    // the flag whether in opne list
    bool opened_;

    // the flag whether in closed list
    bool closed_;

    // the predecessor pointer
    const Node3D* pred_;
};
} // namespace kindynamic_path_planner
#endif