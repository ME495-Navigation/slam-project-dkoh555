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

    // DiffDrive Getter functions
    Transform2D DiffDrive::get_position() const {
        return robot_position;
    }

    WheelPosition DiffDrive::get_wheels() const {
        return wheel_positions;
    }

    // DiffDrive functions
    void DiffDrive::forward_k(WheelPosition new_positions) {
        // Note the change in wheel position
        WheelPosition delta_wheel{new_positions.right -wheel_positions.right,
            new_positions.left -wheel_positions.left};

        // Calculte the x velocity
        double x_vel = (delta_wheel.right + delta_wheel.left) * wheel_radius / 2;

        // The y velocity is always 0
        double y_vel = 0.0;

        // Calculate the angular velocity
        double omega_vel = (delta_wheel.right - delta_wheel.left) * wheel_radius / track_width;

        // Using the resulting twist, find the change in position
        Twist2D new_twist{omega_vel, x_vel, y_vel};
        Transform2D new_transform = integrate_twist(new_twist);

        // Apply the change to the current robot configuration
        Vector2D new_position = robot_position.translation() + new_transform.translation();
        double new_rotation = robot_position.rotation() + new_transform.rotation();
        Transform2D combined_transform{new_position, new_rotation};
        robot_position = combined_transform;
    }
}