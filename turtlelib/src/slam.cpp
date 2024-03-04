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
        q_size = static_cast<arma::uword>(new_q_size);
        max_landmarks = static_cast<arma::uword>(new_max_landmarks);
        W_noise = new_W_noise;
        R_noise = new_R_noise;
    }

    //
    // Functions
    //
    void Slam::initialize_sigma_t()
    {
        arma::mat sigma_zero_q{q_size, q_size, arma::fill::zeros};
        arma::mat sigma_zero_m = arma::zeros(2 * max_landmarks, 2 * max_landmarks) * HIGH_UNCERTAINTY;
        arma::mat zero12{q_size, 2 * max_landmarks, arma::fill::zeros};
        arma::mat zero21{2 * max_landmarks, q_size, arma::fill::zeros};

        sigma_t = arma::join_vert(arma::join_horiz(sigma_zero_q, zero12), arma::join_horiz(zero21, sigma_zero_m));
    }

    void Slam::update_xi()
    {
        xi_t = arma::join_vert(q_t, m_t);
    }

    void Slam::update_q_t_m_t()
    {
        q_t = xi_t.subvec(0, q_size - 1);
        m_t = xi_t.subvec(q_size, q_size + 2 * max_landmarks - 1);
    }

    void Slam::set_q_t(Transform2D robot_position)
    {
        q_t(0) = robot_position.rotation();
        q_t(1) = robot_position.translation().x;
        q_t(2) = robot_position.translation().y;
    }
}