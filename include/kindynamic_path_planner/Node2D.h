#ifndef KINDYNAMIC_PATH_PLANNER_NODE2D_H_
#define KINDYNAMIC_PATH_PLANNER_NODE2D_H_

namespace kindynamic_path_planner {
class Node2D {
public:
    // x, y, g, h, pred
    Node2D() : Node2D(0, 0, .0, .0, nullptr) {}
    Node2D(int x, int y, double g, double h, Node2D* pred) {
        this->index_x = x;
        this->index_y = y;
        this->g_ = g;
        this->h_ = h;
        this->pred_ = pred;
        this->open = false;
        this->closed = false;
        this->idx_ = -1;
    }

    int getX() const { return this->index_x; }
    void setX(const int x) { this->index_x = x; }

    int getY() const { return this->index_y; }
    void setY(const int y) { this->index_y = y; }

    double getG() const { return this->g_; }
    void setG(const double g) { this->g_ = g; }

    double getH() const { return this->h_; }
    void setH(const double h) { this->h_ = h; }

    double getC() const { return this->g_ + this->h_; }

    // double getF() const { return this->g_ + this->h_; }

    int getIdx() const { return this->idx_; }
    int setIdx(int width) {
        this->idx_ = index_x + index_y * width;
        return idx_;
    }

    bool isInOpenedList() { return this->open; }
    void putInOpenedList() {
        this->open = true;
        this->closed = false;
    }

    bool isInClosedList() { return this->closed; }
    void putInClosedList() {
        this->open = false;
        this->closed = true;
    }

    const Node2D* getPred() const { return this->pred_; }
    void setPred(Node2D* pred) { this->pred_ = pred; }

    Node2D* createSuccssor(const int i);

    void reset() {
        this->open = false;
        this->closed = false;
    }

    /// Custom operator to compare nodes. Nodes are equal if their x and y position is the same.
    bool operator==(const Node2D& rhs) const;

    static const int dir;

    static const int dx[];

    static const int dy[];

private:
    // idx in grid map x
    int index_x;
    int index_y;
    int idx_;
    double g_;
    double h_;
    bool open;
    bool closed;

    const Node2D* pred_;
};
} // namespace kindynamic_path_planner
#endif
