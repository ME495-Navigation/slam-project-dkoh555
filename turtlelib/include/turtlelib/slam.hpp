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
    /// \class Slam
    /// \brief Implements Extended Kalman Filter SLAM for a differential drive robot.
    ///
    /// This class maintains the robot's pose, map of landmarks, and provides methods
    /// for initializing the SLAM process, predicting robot motion, and correcting
    /// the pose and map based on sensor measurements.
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
        /// \brief Initializes the SLAM process with default parameters.
        /// \details This constructor sets up the SLAM with a default pose at the origin, default noise levels, and initializes the sigma matrix.
        Slam();

        /// \brief Initializes the SLAM process with a specific robot position.
        /// \param robot_position The starting position of the robot.
        /// \details This constructor allows setting the initial robot pose while keeping other parameters at their default values.
        Slam(Transform2D robot_position);

        /// \brief Initializes the SLAM process with a specific robot position.
        /// \param turtle_pose_0 The starting position of the robot.
        /// \details This constructor allows setting the initial robot pose while keeping other parameters at their default values.
        Slam(Pose2D turtle_pose_0);

        /// \brief Initializes the SLAM process with full configuration.
        /// \param robot_position The starting position of the robot.
        /// \param new_q_size The size of the pose state vector.
        /// \param new_max_landmarks The maximum number of landmarks.
        /// \param new_W_noise The process noise for the robot's motion model.
        /// \param new_R_noise The measurement noise for the robot's sensor model.
        /// \details This constructor allows full customization of the initial SLAM setup including pose, landmark capacity, and noise levels.
        Slam(Transform2D robot_position, int new_q_size, int new_max_landmarks, double new_W_noise, double new_R_noise);
            
        //
        // Functions
        //
        /// \brief Sets the initial pose of the robot from a Pose2D object.
        /// \param turtle_pose_0 The initial pose of the robot.
        /// \details This function initializes the robot's pose and updates the combined state vector accordingly.
        void initialize_pose(Pose2D turtle_pose_0);

        /// \brief Initializes the 'guess' values for the covariance matrix, sigma_t.
        /// \details Sets the initial uncertainty in the robot's pose and the map, preparing the SLAM for updates.
        void initialize_sigma_t();

        /// \brief Updates the combined state vector from the current pose and map vectors.
        /// \details This function consolidates the pose and map into a single state vector for SLAM processing.
        void update_xi();

        /// \brief Updates the pose and map vectors from the current combined state vector.
        /// \details After adjustments to the combined state, this function applies those changes back to the individual pose and map vectors.
        void update_q_t_m_t();

        /// \brief Sets the current state of the robot.
        /// \param config The new pose of the robot as a Transform2D object.
        /// \details Directly updates the robot's pose within the SLAM process, adjusting the combined state vector as necessary.
        void set_q_t(Transform2D config);

        /// \brief Predicts and updates the estimate of the combined state vector based on input motion.
        /// \param input The motion twist applied to the robot.
        /// \details Based on the given twist, this function predicts the next state of the robot and updates the SLAM state.
        void predict_and_update_xi(Twist2D input);

        /// \brief Propagates the uncertainty of the prediction and updates the sigma matrix.
        /// \details After a motion prediction, this function recalculates the uncertainty in the robot's pose and map.
        void propogate_and_update_sigma();

        /// \brief Corrects the combined SLAM state with a new landmark measurement.
        /// \param x The x-coordinate of the landmark measurement.
        /// \param y The y-coordinate of the landmark measurement.
        /// \param landmark_id The ID of the landmark.
        /// \details Incorporates a new measurement into the SLAM state, adjusting the robot's pose and map to better fit observed data.
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

    /// \brief Computes the twist vector necessary to achieve a specified transformation.
    /// \param transform The target transformation to be achieved.
    /// \return A Twist2D object representing the required twist (velocity and rotation) to achieve the given transformation.
    /// \details This function calculates the differential motion (in terms of linear and angular velocity) needed to attain a transformation from the current robot state to the specified target state. It accounts for both translation and rotation, returning the minimal twist that realizes the transformation.
    Twist2D twist_from_transform(const Transform2D& transform);
}

#endif