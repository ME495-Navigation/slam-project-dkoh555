#include "turtlelib/slam.hpp"

namespace turtlelib
{
    //
    // Constructors
    //
    Slam::Slam() {};

    Slam::Slam(Transform2D robot_position)
    {
        set_q_t(robot_position);
    }

    Slam::Slam(Transform2D robot_position, int new_q_size, int new_max_landmarks, double new_W_noise, double new_R_noise)
    {
        set_q_t(robot_position);
        q_size = new_q_size;
        max_landmarks = new_max_landmarks;
        W_noise = new_W_noise;
        R_noise = new_R_noise;
    }

    //
    // Functions
    //
    void Slam::set_q_t(Transform2D robot_position)
    {
        q_t(0) = robot_position.rotation();
        q_t(1) = robot_position.translation().x;
        q_t(2) = robot_position.translation().y;
    }
}