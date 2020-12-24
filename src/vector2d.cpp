#include "kindynamic_path_planner/vector2d.h"

#include <assert.h>

namespace kindynamic_path_planner {
Vector2D Vector2D::unitVector2d(const double angle) {
    Vector2D ans = Vector2D(cos(angle), sin(angle));
    return ans;
}

double Vector2D::norm() const {
    double ans = hypot(x_, y_);
    return ans;
}

double Vector2D::squreNorm() const {
    double ans = x_ * x_ + y_ * y_;
    return ans;
}

double Vector2D::angle() const {
    double ans = std::atan2(y_, x_);
    return ans;
}

void Vector2D::normlize() {
    const double l = norm();

    if (l > kMathEpsilon) {
        x_ /= l;
        y_ /= l;
    }
}

double Vector2D::distanceTo(const Vector2D& other) const {
    double ans = hypot(x_ - other.x_, y_ - other.y_);
    return ans;
}

double Vector2D::squreDistanceTo(const Vector2D& other) const {
    const double dx = x_ - other.x_;
    const double dy = y_ - other.y_;

    return dx * dx + dy * dy;
}

double Vector2D::cross(const Vector2D& other) const {
    double ans = x_ * other.y_ + y_ * other.x_;
    return ans;
}

double Vector2D::dot(const Vector2D& other) const {
    double ans = x_ * other.x_ + y_ * other.y_;

    return ans;
}

Vector2D Vector2D::operator+(const Vector2D& other) const {
    Vector2D ans = Vector2D(x_ + other.x_, y_ + other.y_);

    return ans;
}

Vector2D Vector2D::operator-(const Vector2D& other) const {
    Vector2D ans = Vector2D(x_ - other.x_, y_ - other.y_);

    return ans;
}

Vector2D Vector2D::operator-() const {
    Vector2D ans = Vector2D(-x_, -y_);

    return ans;
}

Vector2D Vector2D::operator*(const double ratio) const {
    Vector2D ans = Vector2D(x_ * ratio, y_ * ratio);

    return ans;
}

Vector2D Vector2D::operator/(const double ratio) const {
    assert(std::abs(ratio) > kMathEpsilon);

    return Vector2D(x_ / ratio, y_ / ratio);
}

Vector2D& Vector2D::operator+=(const Vector2D& other) {
    x_ += other.getX();
    y_ += other.getY();
    return *this;
}

Vector2D& Vector2D::operator-=(const Vector2D& other) {
    x_ -= other.getX();
    y_ -= other.getY();
    return *this;
}

Vector2D& Vector2D::operator*=(const double ratio) {
    x_ *= ratio;
    y_ *= ratio;
    return *this;
}

Vector2D& Vector2D::operator/=(const double ratio) {
    assert(std::abs(ratio) > kMathEpsilon);
    x_ /= ratio;
    y_ /= ratio;
    return *this;
}

bool Vector2D::operator==(const Vector2D& other) const {
    bool ans = (std::abs(x_ - other.getX()) < kMathEpsilon && std::abs(y_ - other.getY()) < kMathEpsilon);

    return ans;
}

Vector2D operator*(const double ratio, const Vector2D& vec) { return vec * ratio; }

Vector2D Vector2D::ort(Vector2D b) const {
    Vector2D a(this->x_, this->y_);
    Vector2D c;
    // multiply b by the dot product of this and b then divide it by b's length
    c = a - b * a.dot(b) / b.squreNorm();
    return c;
}
} // namespace kindynamic_path_planner
