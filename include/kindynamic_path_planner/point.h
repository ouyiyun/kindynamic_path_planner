#ifndef KINDYNAMIC_PATH_PLANNER_VOROPOINT_H_
#define KINDYNAMIC_PATH_PLANNER_VOROPOINT_H_

#define INTPOINT IntPoint

/*! A light-weight integer point with fields x,y */
class IntPoint {
public:
    IntPoint() : x(0), y(0) {}
    IntPoint(int _x, int _y) : x(_x), y(_y) {}
    int x, y;
};

#endif
