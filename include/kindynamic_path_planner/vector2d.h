#ifndef KINDYNAMIC_PATH_PLANNER_VECTOR2D_H_
#define KINDYNAMIC_PATH_PLANNER_VECTOR2D_H_

#include <cmath>
#include <iostream>

namespace kindynamic_path_planner {
const double kMathEpsilon = 1e-10;
//! A class describing a simple 2D vector
class Vector2D {
public:
    //! default constructor.
    Vector2D(const double x, const double y) : x_(x), y_(y) {}

    //! Constructor returning the zero vector.
    Vector2D() : Vector2D(0., 0.) {}

    //! Creates a unit-vector with a given angle to the positive x semi-axis
    static Vector2D unitVector2d(const double angle);

    //! get x and y
    double getX() const { return x_; }
    double getY() const { return y_; }

    //! set x and y
    void setX(const double x) { x_ = x; }
    void setY(const double y) { y_ = y; }
    void setXY(const double x, const double y) {
        x_ = x;
        y_ = y;
    }

    //! norm: length
    double norm() const;

    //! squre norm
    double squreNorm() const;

    //! Gets the angle between the vector and the positive x semi-axis
    double angle() const;

    //! Returns the unit vector that is co-linear with this vector
    void normlize();

    //! Returns the distance to the given vector
    double distanceTo(const Vector2D& other) const;

    //! Returns the squared distance to the given vector
    double squreDistanceTo(const Vector2D& other) const;

    //! Returns the "cross" product between these two Vector2D (non-standard).
    double cross(const Vector2D& other) const;

    //! Returns the inner product between these two Vector2D.
    double dot(const Vector2D& other) const;

    //! add
    Vector2D operator+(const Vector2D& other) const;

    //! subtract
    Vector2D operator-(const Vector2D& other) const;

    //! negate a vector
    Vector2D operator-() const;

    //! Multiplies
    Vector2D operator*(const double ratio) const;

    //! Divides Vector2D by a scalar
    Vector2D operator/(const double ratio) const;

    //! Sums another Vector2D to the current one
    Vector2D& operator+=(const Vector2D& other);

    //! Subtracts another Vector2D to the current one
    Vector2D& operator-=(const Vector2D& other);

    //! Multiplies this Vector2D by a scalar
    Vector2D& operator*=(const double ratio);

    //! Divides this Vector2D by a scalar
    Vector2D& operator/=(const double ratio);

    //! Compares two Vector2D
    bool operator==(const Vector2D& other) const;

    //! a method that returns the orthogonal complement of two vectors
    Vector2D ort(Vector2D b) const;

private:
    double x_;
    double y_;
};
//! Multiplies the given Vector2D by a given scalar
Vector2D operator*(const double ratio, const Vector2D& vec);
} // namespace kindynamic_path_planner

#endif