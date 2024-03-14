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