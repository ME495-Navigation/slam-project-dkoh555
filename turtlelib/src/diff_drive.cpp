#include "turtlelib/diff_drive.hpp"
#include <cmath>
#include <ostream>
#include <iostream>

namespace turtlelib {
    
    // DiffDrive constructors
    DiffDrive::DiffDrive() :  wheel_radius(0.5), track_width(2.0),
        robot_position(Vector2D{0.0, 0.0}, 0.0), wheel_positions{0.0, 0.0} {
    }

    DiffDrive::DiffDrive(double wheel_radius, double track_width) : 
        wheel_radius(wheel_radius), track_width(track_width),
        robot_position(Vector2D{0.0, 0.0}, 0.0), wheel_positions{0.0, 0.0} {
    }

    DiffDrive::DiffDrive(double wheel_radius, double track_width,
                Transform2D robot_position, WheelPosition wheel_positions) : 
        wheel_radius(wheel_radius), track_width(track_width),
        robot_position(robot_position), wheel_positions(wheel_positions) {
    }

    // Getter functions
    Transform2D DiffDrive::get_position() const {
        return robot_position;
    }

    WheelPosition DiffDrive::get_wheels() const {
        return wheel_positions;
    }
}