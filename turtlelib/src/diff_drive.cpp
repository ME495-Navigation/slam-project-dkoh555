#include "turtlelib/diff_drive.hpp"
#include <cmath>
#include <ostream>
#include <iostream>

namespace turtlelib {
    
    // DiffDrive constructors
    DiffDrive::DiffDrive() :  wheel_radius(0.5), track_width(2.0),
        robot_position(Vector2D{0.0, 0.0}, 0.0), wheel_positions{0.0, 0.0}, robot_twist{} {
    }

    DiffDrive::DiffDrive(double wheel_radius, double track_width) : 
        wheel_radius(wheel_radius), track_width(track_width),
        robot_position(Vector2D{0.0, 0.0}, 0.0), wheel_positions{0.0, 0.0}, robot_twist{} {
    }

    DiffDrive::DiffDrive(double wheel_radius, double track_width,
                Transform2D robot_position, WheelPosition wheel_positions) : 
        wheel_radius(wheel_radius), track_width(track_width),
        robot_position{Vector2D{robot_position.translation().x, robot_position.translation().y}, normalize_angle(robot_position.rotation())},
        wheel_positions(WheelPosition{normalize_angle(wheel_positions.right), normalize_angle(wheel_positions.left)}), robot_twist{} {
    }

    // DiffDrive Getter functions
    Transform2D DiffDrive::get_position() const {
        return robot_position;
    }

    WheelPosition DiffDrive::get_wheels() const {
        return wheel_positions;
    }

    Twist2D DiffDrive::get_twist() const {
        return robot_twist;
    }


    // DiffDrive Setter functions
    void DiffDrive::set_transform(Transform2D new_transform) {
        robot_position = new_transform;
    }

    // DiffDrive helper functions
    void DiffDrive::normalize_robot_angles() {
        wheel_positions.right = normalize_angle(wheel_positions.right);
        wheel_positions.left = normalize_angle(wheel_positions.left);
        // Transform2D normalized_robot_position{robot_position.translation(),
        //                                         normalize_angle(robot_position.rotation())};
    }
    // DiffDrive functions
    void DiffDrive::forward_k(WheelPosition position_change) {
        // Calculte the x velocity
        double x_vel = (position_change.right + position_change.left) * wheel_radius / 2;

        // The y velocity is always 0
        double y_vel = 0.0;

        // Calculate the angular velocity
        double omega_vel = (position_change.right - position_change.left) * wheel_radius / track_width;

        // Using the resulting twist, find the change in position
        Twist2D curr_twist{omega_vel, x_vel, y_vel};
        Transform2D curr_transform = integrate_twist(curr_twist);

        // Combine the wb and bbnew transforms
        Transform2D final_transform = robot_position * curr_transform;

        // Apply the change to the current robot configuration
        robot_position = final_transform;

        // Update the wheel positions of the robot
        wheel_positions.right = wheel_positions.right + position_change.right;
        wheel_positions.left = wheel_positions.left + position_change.left;

        // Normalize the wheel and body rotation angles of the robot
        normalize_robot_angles();

        // Update the current twist of the robot
        robot_twist = curr_twist;
    }

    WheelPosition DiffDrive::inverse_k(Twist2D twist) {
        // If the y component of twist is non-zero, throw an error
        if (twist.y != 0.0)
        {
            throw std::logic_error("Non-zero y component in twist implies robot slipping.");
        }

        // Calculate the change in wheel positions required to achieve the input twist
        double right_change = (1/wheel_radius) * (twist.x + ((track_width * twist.omega)/2));
        double left_change = (1/wheel_radius) * (twist.x - ((track_width * twist.omega)/2));

        return WheelPosition{right_change, left_change};
    }
}