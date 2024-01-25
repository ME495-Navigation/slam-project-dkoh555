#include "turtlelib/se2d.hpp"
#include <cmath>
#include <ostream>
#include <iostream>

namespace turtlelib {

    // Twist2D operations
    
    std::ostream & operator<<(std::ostream & os, const Twist2D & tw) {
        os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & tw) {
        char ch;
        if (is.peek() == '[') {
            is >> ch >> tw.omega >> tw.x >> tw.y >> ch;
        } else {
            is >> tw.omega >> tw.x >> tw.y;
        }
        return is;
    }

    // Transform2D constructors
    Transform2D::Transform2D() : trans_comp{0.0, 0.0}, rot_comp{0.0} {
    }

    Transform2D::Transform2D(Vector2D trans) : trans_comp{trans.x, trans.y}, rot_comp{0.0} {
    }

    Transform2D::Transform2D(double radians) : trans_comp{0.0, 0.0}, rot_comp{radians} {
    }

    Transform2D::Transform2D(Vector2D trans, double radians) :
        trans_comp{trans.x, trans.y}, rot_comp{radians} {
    }

    // Getter functions
    Vector2D Transform2D::translation() const {
        return trans_comp;
    }

    double Transform2D::rotation() const {
        return rot_comp;
    }

    // Transform methods
    Point2D Transform2D::operator()(Point2D p) const {
        Point2D pnt;
        pnt.x = p.x * cos(rot_comp) - p.y * sin(rot_comp);
        pnt.y = p.x * sin(rot_comp) + p.y * cos(rot_comp);
        pnt = pnt + trans_comp;
        return pnt;
    }

    Vector2D Transform2D::operator()(Vector2D v) const {
        Vector2D vec;
        vec.x = v.x * cos(rot_comp) - v.y * sin(rot_comp);
        vec.y = v.x * sin(rot_comp) + v.y * cos(rot_comp);
        return vec;
    }

    Twist2D Transform2D::operator()(Twist2D v) const {
        Twist2D twi;
        twi.x = v.x * cos(rot_comp) - v.y * sin(rot_comp) + trans_comp.y * v.omega;
        twi.y = v.x * sin(rot_comp) + v.y * cos(rot_comp) - trans_comp.x * v.omega;
        twi.omega = v.omega;
        return twi;
    }

    Transform2D Transform2D::inv() const {
        Transform2D transform;
        transform.trans_comp.x = - trans_comp.x * cos(rot_comp) - trans_comp.y * sin(rot_comp);
        transform.trans_comp.y = - trans_comp.y * cos(rot_comp) + trans_comp.x * sin(rot_comp);
        transform.rot_comp = -rot_comp;
        return transform;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs) {
        double x = rhs.trans_comp.x * cos(rot_comp) - rhs.trans_comp.y * sin(rot_comp) + trans_comp.x;
        double y = rhs.trans_comp.x * sin(rot_comp) + rhs.trans_comp.y * cos(rot_comp) + trans_comp.y;
        rot_comp = normalize_angle(rot_comp + rhs.rot_comp);
        trans_comp = Vector2D{x, y};
        return *this;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf) {
        os << "deg: " << rad2deg(tf.rot_comp) << " x: " << tf.trans_comp.x << " y: " << tf.trans_comp.y;
        return os;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf) {
        std::string s1, s2, s3;
        Vector2D vect;
        double rot;

        if(is.peek() == 'd')
        {
            is >> s1;
            is >> rot;
            is >> s2;
            is >> vect.x;
            is >> s3;
            is >> vect.y;
        }
        else
        {
            is >> rot >> vect.x >> vect.y;
        }
        
        tf = Transform2D(vect, deg2rad(rot));
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs) {
        lhs *= rhs;
        return lhs;
    }

}