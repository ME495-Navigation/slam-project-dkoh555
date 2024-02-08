#ifndef TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
#define TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include<iosfwd> // contains forward definitions for iostream objects

#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"

namespace turtlelib
{
    /// \brief represent the positions of set of wheels
    struct WheelPosition
    {
        /// \brief the right wheel position
        double right = 0.0;

        /// \brief the left wheel position
        double left = 0.0;

        // Constructors
        /// \brief Default constructor
        WheelPosition() = default;
    };

    /// \brief a rigid body transformation in 2 dimensions
    class DiffDrive
    {
        private:
            /// \brief the robot's wheel radius
            double wheel_radius;
            /// \brief the distance between the robot's wheels
            double track_width;
            /// \brief the robot's position configuration
            Transform2D robot_position;
            /// \brief the robot's wheel positions in radians {right, left}
            WheelPosition wheel_positions;
            /// \brief the robot's current twist
            Twist2D robot_twist;
        
        public:
            // Constructors
            /// \brief Create a default diff drive robot
            DiffDrive();

            /// \brief Create a diff drive robot with basic dimensions specified 
            /// \param wheel_radius - robot's wheel radius
            /// \param track_width - robot's track width
            explicit DiffDrive(double wheel_radius, double track_width);

            /// \brief Create a fully detailed diff drive robot
            /// \param wheel_radius - robot's wheel radius
            /// \param track_width - robot's track width
            /// \param robot_position - robot's starting position
            /// \param wheel_positions - robot's starting wheel positions
            DiffDrive(double wheel_radius, double track_width,
                Transform2D robot_position, WheelPosition wheel_positions);

            // Getter Functions
            /// \brief Return the robot's current position configuration
            Transform2D get_position() const;

            /// \brief Return the robot's current wheel positions
            WheelPosition get_wheels() const;

            /// \brief Return the robot's current twist
            Twist2D get_twist() const;

            // Setter Functions
            void set_position(Transform2D new_position);

            // Helper Functions
            /// \brief Normalizes the angle values of the robot's wheel positions and body rotation
            void normalize_robot_angles();

            // Functions
            /// \brief Updates robot's position and wheel configuration based on new wheel positions
            /// \param new_positions - robot's new wheel positions (radians)
            void forward_k(WheelPosition position_change);

            /// \brief Returns the wheel position change required to attain a provided twist
            /// \param twist - robot's linear and angular velocities
            WheelPosition inverse_k(Twist2D twist);
    };
}

#endif
