#include "turtlelib/geometry2d.hpp" // Replace with the actual path of your header file

namespace turtlelib {

    double normalize_angle(double rad) {
        while (rad > PI) rad -= 2 * PI;
        while (rad <= -PI) rad += 2 * PI;
        return rad;
    }

}