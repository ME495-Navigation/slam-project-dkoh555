#include "turtlelib/geometry2d.hpp"
#include <math.h>
#include <ostream>
#include <iostream>

namespace turtlelib {

    double normalize_angle(double rad) {
        return atan2(sin(rad), cos(rad));
    }

    std::ostream & operator<<(std::ostream & os, const Point2D & p) {
        os << "[" << p.x << " " << p.y << "]";
        return os;
    }

    std::istream& operator>>(std::istream& is, Point2D& p) {
        char ch;
        if (is.peek() == '[') {
            is >> ch >> p.x >> p.y >> ch;
        } else {
            is >> p.x >> p.y;
        }
        return is;
    }

    Vector2D operator+(const Vector2D & v1, const Vector2D & v2) {
        Vector2D vect;
        vect.x = v1.x + v2.x;
        vect.y = v1.y + v2.y;
        return vect;
    }

    Vector2D & Vector2D::operator+=(const Vector2D & addition) {
        x = x + addition.x;
        y = y + addition.y;
        return *this;
    }

    Vector2D operator-(const Vector2D & v1, const Vector2D & v2) {
        Vector2D vect;
        vect.x = v1.x - v2.x;
        vect.y = v1.y - v2.y;
        return vect;
    }

    Vector2D & Vector2D::operator-=(const Vector2D & subtractor) {
        x = x + subtractor.x;
        y = y + subtractor.y;
        return *this;
    }

    Vector2D operator*(const Vector2D & v1, const Vector2D & v2) {
        Vector2D vect;
        vect.x = v1.x * v2.x;
        vect.y = v1.y * v2.y;
        return vect;
    }

    Vector2D & Vector2D::operator*=(const Vector2D & multiplier) {
        x = x * multiplier.x;
        y = y * multiplier.y;
        return *this;
    }

    double dot(const Vector2D & v1, const Vector2D & v2) {
        return (v1.x * v2.x + v1.y * v2.y);
    }

    double magnitude(const Vector2D & vector) {
        return sqrt(vector.x * vector.x + vector.y * vector.y);
    }

    double angle(const Vector2D & v1, const Vector2D & v2) {
        return atan2(v2.x - v1.x, v2.y - v1.y);
    }

    Vector2D operator-(const Point2D & head, const Point2D & tail) {
        Vector2D vect;
        vect.x = head.x - tail.x;
        vect.y = head.y - tail.y;
        return vect;
    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp) {
        Point2D pnt;
        pnt.x = tail.x + disp.x;
        pnt.y = tail.y + disp.y;
        return pnt;
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    std::istream& operator>>(std::istream & is, Vector2D & v) {
        char ch;
        if (is.peek() == '[') {
            is >> ch >> v.x >> v.y >> ch;
        } else {
            is >> v.x >> v.y;
        }
        return is;
    }

    Vector2D normalizeVector(const Vector2D & v) {
    Vector2D v_hat{};
    double v_norm = sqrt(v.x * v.x + v.y * v.y);

    if(v_norm == 0.0)
    {
        std::cout << "Invalid vector" << std::endl;
    }
    else
    {
        v_hat.x = v.x / v_norm;
        v_hat.y = v.y / v_norm;
    }

    return v_hat;
}

}