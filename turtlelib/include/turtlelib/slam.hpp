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

#define HIGH_UNCERTAINTY 1e6

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
        arma::mat Q{arma::eye(q_size, q_size) * W_noise};
        /// \brief Set of previously seen landmarks by ID number
        std::unordered_set<int> seen_landmarks;
        /// \brief Actual measurement of features in the form of range and bearing, [r1 phi1 r2 phi2 ... rn phin]'
        arma::colvec zi_t{2, arma::fill::zeros};
        /// \brief Estimate of feature locations in the form of range and bearing based on robot pose change, [r1 phi1 r2 phi2 ... rn phin]'
        arma::colvec zi_t_hat{2, arma::fill::zeros};
        /// \brief H matrix
        arma::mat Hi_t{2, q_size + 2 * max_landmarks, arma::fill::zeros};
        /// \brief Sensor noise matrix
        arma::mat R{2, 2, arma::fill::eye};
        /// \brief Kalman gain matrix
        arma::mat Ki_t{3 + 2 * max_landmarks, 2, arma::fill::zeros};


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

        Slam(Pose2D turtle_pose_0);
            
        //
        // Functions
        //
        void initialize_pose(Pose2D turtle_pose_0);

        /// \brief Initialize the initial 'guess' values for sigma_t
        void initialize_sigma_t();

        /// \brief Uses current q_t and m_t vectors to update xi_t
        void update_xi();

        /// \brief Uses current xi_t vector to update q_t and m_t
        void update_q_t_m_t();

        /// \brief Set the current state, q_t, of the robot
        void set_q_t(Transform2D config);

        /// \brief Predict and update the estimate of the combined state vector, xi_t (equation 20 implementation)
        void predict_and_update_xi(Twist2D input);

        /// \brief Propogate the uncertainty of the prediction (equation 21 implementation), to be used after predict_and_update_xi()
        void propogate_and_update_sigma();

        /// \brief Correct the combined state with every new landmark measurement (equation 22 implementation)
        void correct_with_landmark(double x, double y, int landmark_id);
        
        //
        // Getters
        //
        Transform2D get_transform()
        {
            return Transform2D{Vector2D{q_t(1), q_t(2)}, q_t(0)};
        }

        Pose2D pose() const
        {
            return Pose2D{q_t(0), q_t(1), q_t(2)};
        }

        arma::colvec map() const
        {
            return m_t;
        }

        arma::colvec state_vector() const
        {
            return xi_t;
        }

        arma::mat covariance_matrix() const
        {
            return sigma_t;
        }

        Twist2D twist() const
        {
            return Twist2D{u_t(0), u_t(1), u_t(2)};
        }
        
        arma::mat state_matrix() const
        {
            return A;
        }

        arma::mat actual_measurement() const
        {
            return zi_t;
        }

        arma::mat predicted_measurement() const
        {
            return zi_t_hat;
        }
        arma::mat sensor_matrix() const
        {
            return Hi_t;
        }
    };

    /// \brief Returns the twist required to attain a given transformation
    Twist2D twist_from_transform(const Transform2D& transform);
}

#endif