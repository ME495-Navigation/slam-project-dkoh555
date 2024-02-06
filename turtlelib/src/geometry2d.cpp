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
        char ch; // uninitialized
        if (is.peek() == '[') {
            is >> ch >> p.x >> p.y >> ch;
        } else {
            is >> p.x >> p.y;
        }
        return is;
    }

    Vector2D operator-(const Point2D & head, const Point2D & tail) {
        Vector2D vect;
        vect.x = head.x - tail.x;
        vect.y = head.y - tail.y;
        return vect; // return {head.x - tail.x, head.y - tail.y}
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
        char ch;//uninitialized
        if (is.peek() == '[') {
            is >> ch >> v.x >> v.y >> ch;
        } else {
            is >> v.x >> v.y;
        }
        return is;
    }

    Vector2D normalizeVector(const Vector2D & v) {
    Vector2D v_hat{};
    double v_norm = sqrt(v.x * v.x + v.y * v.y); // const auto

    if(v_norm == 0.0)
    {
        std::cout << "Invalid vector" << std::endl;
        // this prints an error but the function still returns the same vector meaning the code has not much to go on
        // if passing this. could throw an exception or divide by zero
    }
    else
    {
        v_hat.x = v.x / v_norm;
        v_hat.y = v.y / v_norm;
    }

    return v_hat;
}

}
