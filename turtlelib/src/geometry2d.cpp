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

    Vector2D operator-(const Point2D & head, const Point2D & tail) {
        Vector2D vect;
        vect.x = head.x - tail.x;
        vect.y = head.y - tail.y;
        return vect;
    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp) {
        Point2D pnt;
        pnt.x = tail.x - disp.x;
        pnt.y = tail.y - disp.y;
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

}