#include "kindynamic_path_planner/Node2D.h"

namespace kindynamic_path_planner {
//! possible directions
const int Node2D::dir = 8;

//! possible move direction
/*
 * 0 1 2
 * 3   4
 * 5 6 7
 */
const int Node2D::dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};

const int Node2D::dy[] = {1, 1, 1, 0, 0, -1, -1, -1};

Node2D* Node2D::createSuccssor(const int i) {
    int xSucc = this->index_x + dx[i];
    int ySucc = this->index_y + dy[i];

    return new Node2D(xSucc, ySucc, g_, 0, this);
}

bool Node2D::operator==(const Node2D& rhs) const { return index_x == rhs.index_x && index_y == rhs.index_y; }
} // namespace kindynamic_path_planner