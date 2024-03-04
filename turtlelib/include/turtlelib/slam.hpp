#ifndef TURTLESIM_SLAM_INCLUDE_GUARD_HPP
#define TURTLESIM_SLAM_INCLUDE_GUARD_HPP
/// \file
/// \brief 

#include <iosfwd>
#include <cmath>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <armadillo>
#include <unordered_set>

#define HIGH_UNCERTAINTY 1e7

namespace turtlelib
{
    /// \brief The EKF SLAM implementation for a differential drive robot
    class Slam
    {
    private:
        //
        // Modifiable Variables (Can modify for other robots or scenarios)
        //
        /// \brief Number of elements in column vector q_t
        arma::uword q_size = 3;
        /// \brief Maximum number of landmarks in the map (affects dimensions of other vectors)
        arma::uword max_landmarks = 10;
        /// \brief Process noise for the robot's motion model (variance)
        double W_noise = 0.001;
        /// \brief Measurement noise for the robot's sensor model for landmarks (variance)
        double R_noise = 0.01;

        //
        // Key Variables
        //
        /// \brief Current state of the robot at time t, [theta x y]'
        arma::colvec q_t{q_size, arma::fill::zeros};
        /// \brief Current state of the map at time t, [m_x1 m_y1 m_x2 m_y2 ... m_xn m_yn]'
        arma::colvec m_t{2 * max_landmarks, arma::fill::zeros};
        /// \brief Current combined state vector at time t, [q_t m_t]'
        arma::colvec xi_t{q_size + 2 * max_landmarks, arma::fill::zeros};
        /// \brief Current twist of the robot at time t, [dtheta dx dy]'
        arma::colvec u_t{q_size, arma::fill::zeros};

        //
        // Additional Variables (Rely on other members and variables)
        //
        /// \brief The Identity matrix of size (q_size + 2 * max_landmarks) x (q_size + 2 * max_landmarks), used to calculate A
        arma::mat I = arma::eye(q_size + 2 * max_landmarks, q_size + 2 * max_landmarks);
        /// \brief The covariance matrix of size (q_size + 2 * max_landmarks) x (q_size + 2 * max_landmarks), needs to be initialized
        arma::mat sigma_t{q_size + 2 * max_landmarks, q_size + 2 * max_landmarks, arma::fill::zeros};
        /// \brief The linearized state transition matrix for propogating uncertainty, used to calculate sigma_t
        arma::mat A = arma::eye(q_size + 2 * max_landmarks, q_size + 2 * max_landmarks);
        /// \brief The process noise matrix for the robot motion model, used to calculate sigma_t
        arma::mat Q_bar{arma::eye(q_size, q_size) * W_noise};


    public:
        //
        // Constructors
        //
        /// \brief Default constructor where SLAM starts at origin with default key variables
        Slam();

        /// \brief Constructor where SLAM starts with provided configuration and default key variables
        Slam(Transform2D robot_position);

        /// \brief Constructor where SLAM starts with provided configuration and provided key variables
        Slam(Transform2D robot_position, int new_q_size, int new_max_landmarks, double new_W_noise, double new_R_noise);

        //
        // Functions
        //
        /// \brief Initialize the initial 'guess' values for sigma_t
        void initialize_sigma_t();

        /// \brief Uses current q_t and m_t vectors to update xi_t
        void update_xi();

        /// \brief Uses current xi_t vector to update q_t and m_t
        void update_q_t_m_t();

        /// \brief Set the current state, q_t, of the robot
        void set_q_t(Transform2D config);

        // /// \brief Predict and update the estimate of the combined state vector, xi_t (equation 20 implementation)
        // arma::colvec predict_xi(arma::colvec u_t);

        // /// \brief Propogate the uncertainty of the prediction (equation 21 implementation)
        // arma::colvec propogate_uncertainty();
    };
}

#endif