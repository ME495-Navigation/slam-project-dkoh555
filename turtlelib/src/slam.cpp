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
        arma::mat zeros12{q_size, 2 * max_landmarks, arma::fill::zeros};
        arma::mat zeros21{2 * max_landmarks, q_size, arma::fill::zeros};

        sigma_t = arma::join_vert(arma::join_horiz(sigma_zero_q, zeros12), arma::join_horiz(zeros21, sigma_zero_m));
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

    void Slam::predict_and_update_xi(Twist2D input)
    {
        // Ensure delta_y of input is 0
        if(!almost_equal(input.y, 0.0))
        {
            throw std::runtime_error("delta_y of input twist must be 0.0");    
        }

        // Update Slam u_t
        u_t(0) = input.omega;
        u_t(1) = input.x;
        u_t(2) = 0.0;

        // Predict the new robot state given the input
        // tf from world to current robot position
        Transform2D tf_world_robot{Vector2D{q_t(1), q_t(2)}, q_t(0)};
        // tf from current robot to predicted robot position
        Transform2D tf_robot_predrobot = integrate_twist(input);
        // tf from world to predicted robot position
        Transform2D tf_world_predrobot = tf_world_robot * tf_robot_predrobot;

        // Update Slam q_t with predicted robot position
        set_q_t(tf_world_predrobot);
        // Update Slam xi_t
        update_xi();
    }

    void Slam::propogate_and_update_sigma()
    {
        // Find A
        // Initialize variables for propogation
        arma::mat pose_state_mat(q_size, q_size, arma::fill::zeros);
        arma::mat zeros12{q_size, 2 * max_landmarks, arma::fill::zeros};
        arma::mat zeros21{2 * max_landmarks, q_size, arma::fill::zeros};
        arma::mat zeros22{2 * max_landmarks, 2 * max_landmarks, arma::fill::zeros};

        // Check the rotational velocity of Slam u_t, then implement the appropriate equation
        if(almost_equal(u_t(0), 0.0))
        {
            // Implement equation 9
            pose_state_mat(1, 0) = -u_t(1) * sin(q_t(0));
            pose_state_mat(2, 0) = u_t(1) * cos(q_t(0));
        }
        else
        {
            // Implement equation 10
            pose_state_mat(1, 0) = -(u_t(1) / u_t(0)) * cos(q_t(0) + (u_t(1) / u_t(0)) * cos(normalize_angle(q_t(0) + u_t(0))));
            pose_state_mat(2, 0) = -(u_t(1) / u_t(0)) * sin(q_t(0) + (u_t(1) / u_t(0)) * sin(normalize_angle(q_t(0) + u_t(0))));
        }

        arma::mat rhs = arma::join_vert(
            arma::join_horiz(pose_state_mat, zeros12),
            arma::join_horiz(zeros21, zeros22)
        );

        A = I + rhs;

        // Find Q_bar,
        // Implement equation 22
        arma::mat Q_bar = arma::join_vert(
            arma::join_horiz(Q, zeros12),
            arma::join_horiz(zeros21, zeros22)
        );

        // Find the new sigma_t prediction,
        // Implement equation 21
        sigma_t = A * sigma_t * A.t() + Q_bar;
    }

    void Slam::correct_with_landmark(int x, int y, int landmark_id_int)
    {
        // Convert the landmark id to a arma::uword to avoid warning
        arma::uword landmark_id = static_cast<arma::uword>(landmark_id_int);

        // Convert cartensian coordinates to range and bearing
        double range = std::sqrt(std::pow(x - q_t(1), 2) + std::pow(y - q_t(2), 2));
        double bearing = turtlelib::normalize_angle(std::atan2(x, y));

        // Check to see if landmark has been seen before,
        // If not then add it
        if(seen_landmarks.find(landmark_id) == seen_landmarks.end())
        {
            // Add the landmark to the set of seen landmarks
            seen_landmarks.insert(landmark_id);

            // Add the landmark to the predicted map state vector
            // (remember that it is relative to the world frame, not the robot frame)
            m_t(2 * landmark_id) = x + range + cos(bearing + q_t(0));
            m_t(2 * landmark_id + 1) = y + range + sin(bearing + q_t(0));

            // Update the state vector
            update_xi();
        }

        // Note the actual measurement of the feature
        zi_t(0) = range;
        zi_t(1) = bearing;

        // Relative prediction of the feature
        Vector2D landmark_rel_pred{m_t(2 * landmark_id) - q_t(1), m_t(2 * landmark_id + 1) - q_t(2)};
        double rel_pred_sqrd = std::pow(landmark_rel_pred.x, 2) + std::pow(landmark_rel_pred.y, 2); // THIS OKAY?

        // Relative predicion of feature as range and bearing
        double range_hat = std::sqrt(rel_pred_sqrd);
        double bearing_hat = turtlelib::normalize_angle(std::atan2(landmark_rel_pred.y, landmark_rel_pred.x) - q_t(0));
        zi_t_hat(0) = range_hat;
        zi_t_hat(1) = bearing_hat;

        // H matrix calculations
        arma::mat small_H_1{2, q_size, arma::fill::zeros};
        arma::mat zeros_2_1{2, 2 * landmark_id, arma::fill::zeros};
        arma::mat small_H_2{2, 2, arma::fill::zeros};
        arma::mat zeros_2_2{2, 2 * max_landmarks - 2 * (landmark_id + 1), arma::fill::zeros};

        small_H_1(0, 0) = 0.0;
        small_H_1(0, 1) = -landmark_rel_pred.x / std::sqrt(rel_pred_sqrd);
        small_H_1(0, 2) = -landmark_rel_pred.y / std::sqrt(rel_pred_sqrd);
        small_H_1(1, 0) = -1;
        small_H_1(1, 1) = landmark_rel_pred.y / rel_pred_sqrd;
        small_H_1(1, 2) = -landmark_rel_pred.x / rel_pred_sqrd;

        small_H_2(0, 0) = landmark_rel_pred.x / std::sqrt(rel_pred_sqrd);
        small_H_2(0, 1) = landmark_rel_pred.y / std::sqrt(rel_pred_sqrd);
        small_H_2(1, 0) = -landmark_rel_pred.y / rel_pred_sqrd;
        small_H_2(1, 1) = landmark_rel_pred.x / rel_pred_sqrd;

        Hi_t = arma::join_horiz(arma::join_horiz(small_H_1, zeros_2_1), arma::join_horiz(small_H_2, zeros_2_2));

        // Sensor noise matrix calculations
        R = arma::mat{2, 2, arma::fill::eye} * R_noise;
        // Kalman gain matrix calulations
        Ki_t = sigma_t * Hi_t.t() * (Hi_t * sigma_t * Hi_t.t() + R).i();

        // Compare zi_t and zi_t_hat and adjust state accordingly
        arma::colvec zi_diff{2, arma::fill::zeros};
        zi_diff(0) = zi_t(0) - zi_t_hat(0);
        zi_diff(1) = normalize_angle(zi_t(1) - zi_t_hat(1));
        xi_t = xi_t + Ki_t * zi_diff;

        // Update q_t and m_t
        update_q_t_m_t();

        // Update the covariance again
        sigma_t = (I - Ki_t * Hi_t) * sigma_t;
    }

    Twist2D twist_from_transform(const Transform2D& transform)
    {
        // Check for pure linear transformation
        if(almost_equal(transform.rotation(), 0.0) && almost_equal(transform.translation().y, 0.0))
        {
            return Twist2D{0.0, transform.translation().x, 0.0};
        }
        else
        {
            double trans = fabs(transform.translation().y / (1 - cos(transform.rotation())));
            return Twist2D{transform.rotation(), trans * transform.rotation(), 0.0};
        }
    }
}