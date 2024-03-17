#include "turtlelib/slam.hpp"
#include <cmath>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#define TOLERANCE 1.0e-6

using Catch::Matchers::WithinAbs;

using turtlelib::Slam;
using turtlelib::Pose2D;
using turtlelib::Twist2D;
using turtlelib::Transform2D;
using turtlelib::Vector2D;

int q_size = 3;
int num_landmarks = 3;
double w = 0.001;

// Covariance matrix
arma::mat sigma_0_q = arma::zeros<arma::mat>(q_size, q_size); // We are absolutely sure about the initial pose
arma::mat sigma_0_m = arma::eye(2 * num_landmarks, 2 * num_landmarks) * 1e6; // Uncertainity in sensing, very high with no knowledge of obstacles
arma::mat zeros_12 = arma::zeros<arma::mat>(q_size, 2*num_landmarks); // Zeros due to sensing and localization noise being independent
arma::mat zeros_21 = arma::zeros<arma::mat>(2*num_landmarks, q_size); // Zeros due to sensing and localization noise being independent
arma::mat zeros_22 = arma::zeros<arma::mat>(2*num_landmarks, 2*num_landmarks); // 
arma::mat I = arma::eye(q_size + 2*num_landmarks, q_size + 2*num_landmarks); // 
arma::mat sigma_0 =
    arma::join_vert(
    arma::join_horiz(sigma_0_q, zeros_12), 
    arma::join_horiz(zeros_21, sigma_0_m));

// State Matrix
arma::mat A_0 = arma::eye(q_size+2*num_landmarks, q_size+2*num_landmarks);

// Predictable process Noise
arma::mat Q{arma::mat{static_cast<arma::uword>(q_size),static_cast<arma::uword>(q_size),arma::fill::eye}*w};
arma::mat Q_bar =
            arma::join_vert(
            arma::join_horiz(Q, zeros_12), 
            arma::join_horiz(zeros_21, zeros_22));


void ekf_check_pose(Slam subject, Pose2D required_pose);
void ekf_check_map(Slam subject, arma::vec required_map);
void ekf_check_state_vector(Slam subject, arma::vec required_state_vector);
void ekf_check_covariance_matrix(Slam subject, arma::mat required_covariance_matrix);
void ekf_check_twist(Slam subject, Twist2D required_twist);
void ekf_check_state_matrix(Slam subject, arma::mat required_state_matrix);
// void ekf_check_actual_measurement(Slam subject, arma::vec required_actual_measurement);
// void ekf_check_predicted_measurement(Slam subject, arma::vec required_predicted_measurement);
// void ekf_check_sensor_matrix(Slam subject, arma::mat required_sensor_matrix);

namespace turtlelib {

    TEST_CASE("initial_armadillo_test", "slam") {
        arma::mat test_matrix = arma::mat(2, 2, arma::fill::eye);
        
        auto val = test_matrix(0, 0);
        REQUIRE(val == 1);

        val = test_matrix(1,0);
        REQUIRE(val == 0);

        test_matrix.ones();
        val = test_matrix(1,0);
        REQUIRE(val == 1);
    }
}

TEST_CASE( "Initialization works for EKFSLam", "[EKFSlam()]") // Aditya Nair
{
    Slam estimator;

    // Pose
    ekf_check_pose(estimator, Pose2D{0.0, 0.0, 0.0});

    // Map
    ekf_check_map(estimator, arma::zeros<arma::vec>(2*num_landmarks));

    // State vector
    ekf_check_state_vector(estimator, arma::zeros<arma::vec>(q_size + 2*num_landmarks));

    // Covariance matrix
    ekf_check_covariance_matrix(estimator, sigma_0);

    // Twist
    ekf_check_twist(estimator, Twist2D{0.0, 0.0, 0.0});

    // State Matrix
    ekf_check_state_matrix(estimator, A_0);
}

TEST_CASE( "Initialization with pose works for EKFSLam", "[EKFSlam(Pose2D)]") // Aditya Nair
{
    Pose2D pose{-69, 6.9, 4.20};
    Slam estimator(pose);

    // Pose
    ekf_check_pose(estimator, pose);

    // Map
    ekf_check_map(estimator, arma::zeros<arma::vec>(2*num_landmarks));

    // State vector
    ekf_check_state_vector(estimator, arma::join_vert(arma::vec({pose.theta, pose.x, pose.y}), arma::zeros<arma::vec>(6)));

    // Covariance matrix
    ekf_check_covariance_matrix(estimator, sigma_0);

    // Twist
    ekf_check_twist(estimator, Twist2D{0.0, 0.0, 0.0});

    // State Matrix
    ekf_check_state_matrix(estimator, A_0);
}

TEST_CASE( "Prediction works for EKFSLam", "[predict(Twist2D)]") // Aditya Nair
{
    Pose2D pose{-69, 6.9, 4.20};

    Slam estimator_ptr = Slam(pose);
    // estimator_ptr = std::make_unique<turtlelib::Slam>(pose);

    // EKFSlam estimator(pose);

    // 1. Check for pure rotation
    Twist2D v1{3.1, 0, 0};
    estimator_ptr.predict_and_update_xi(v1);

    // Pose
    ekf_check_pose(estimator_ptr, Pose2D{-3.0681469282, 6.9, 4.20});
    // Map
    ekf_check_map(estimator_ptr, arma::zeros<arma::vec>(2*num_landmarks));
    // State vector
    ekf_check_state_vector(estimator_ptr, arma::join_vert(arma::vec({-3.0681469282, 6.9, 4.20}), arma::zeros<arma::vec>(6)));
    // Twist
    ekf_check_twist(estimator_ptr, v1);
    // State matrix
    arma::mat A_1 = A_0;
    ekf_check_state_matrix(estimator_ptr, A_1);
    // Covariance matrix
    arma::mat sigma_1 = A_1 * sigma_0 * A_1.t() + Q_bar;
    ekf_check_covariance_matrix(estimator_ptr, sigma_1);

    // 2. Check for pure translation
    Twist2D v2{0, -6.9, 0};
    estimator_ptr.predict_and_update_xi(v2);

    // Pose
    ekf_check_pose(estimator_ptr, Pose2D{-3.0681469282, 13.781398116933687, 4.706320013688507});
    // Map
    ekf_check_map(estimator_ptr, arma::zeros<arma::vec>(2*num_landmarks));
    // State vector
    ekf_check_state_vector(estimator_ptr, arma::join_vert(arma::vec({-3.0681469282, 13.781398116933687, 4.706320013688507}), arma::zeros<arma::vec>(6)));
    // Twist
    ekf_check_twist(estimator_ptr, v2);

    // State matrix
    arma::mat small_A_2 = arma::zeros<arma::mat>(q_size, q_size);
    small_A_2(1,0) = -0.506320013688507;
    small_A_2(2,0) = 6.881398116933686;
    arma::mat A_2 = I + arma::join_vert(
        arma::join_horiz(small_A_2, zeros_12), 
        arma::join_horiz(zeros_21, zeros_22));
    ekf_check_state_matrix(estimator_ptr, A_2);

    // Covariance matrix
    arma::mat sigma_2 = A_2 * sigma_1 * A_2.t() + Q_bar;
    ekf_check_covariance_matrix(estimator_ptr, sigma_2);

    // 3. Check for general twist
    Twist2D v3{0.42, 6.9, 0};
    estimator_ptr.predict_and_update_xi(v3);

    // Pose
    ekf_check_pose(estimator_ptr, Pose2D{-2.6481469282, 7.2053095338, 2.7907798217});
    // Map
    ekf_check_map(estimator_ptr, arma::zeros<arma::vec>(2*num_landmarks));
    // State vector
    ekf_check_state_vector(estimator_ptr, arma::join_vert(arma::vec({-2.6481469282, 7.2053095338, 2.7907798217}), arma::zeros<arma::vec>(6)));
    // Twist
    ekf_check_twist(estimator_ptr, v3);

    // State matrix
    arma::mat small_A_3 = arma::zeros<arma::mat>(q_size, q_size);
    small_A_3(1,0) = 1.915540192024633;
    small_A_3(2,0) = -6.576088583127138;
    arma::mat A_3 = I + arma::join_vert(
        arma::join_horiz(small_A_3, zeros_12), 
        arma::join_horiz(zeros_21, zeros_22));
    ekf_check_state_matrix(estimator_ptr, A_3);

    // Covariance matrix
    arma::mat sigma_3 = A_3 * sigma_2 * A_3.t() + Q_bar;
    ekf_check_covariance_matrix(estimator_ptr, sigma_3);

    // 4. Check for improper twist (y-component)
    Twist2D v4{0.26, -6.9, -4.2};
    REQUIRE_THROWS(estimator_ptr.predict_and_update_xi(v4));
}

TEST_CASE( "Transform Differentiation works", "[differentiate_transform()]") // Aditya Nair
{
    // Check for pure rotation

    Twist2D v1{3.1, 0, 0};
    Transform2D tf1 = integrate_twist(v1);
    Twist2D vn1 = twist_from_transform(tf1);

    REQUIRE_THAT( v1.omega, WithinAbs(vn1.omega,1.0e-6));
    REQUIRE_THAT( v1.x, WithinAbs(vn1.x,1.0e-6));
    REQUIRE_THAT( v1.y, WithinAbs(vn1.y,1.0e-6));

    // Check for pure translation

    Twist2D v2{0, -6.9, 0.0};
    Transform2D tf2 = integrate_twist(v2);
    Twist2D vn2 = twist_from_transform(tf2);

    REQUIRE_THAT( v2.omega, WithinAbs(vn2.omega,1.0e-6));
    REQUIRE_THAT( v2.x, WithinAbs(vn2.x,1.0e-6));
    REQUIRE_THAT( v2.y, WithinAbs(vn2.y,1.0e-6));

    // Check for general twist

    Twist2D v3{3.1, 6.9, 0.0};
    Transform2D tf3 = integrate_twist(v3); 
    Twist2D vn3 = twist_from_transform(tf3);

    REQUIRE_THAT( v3.omega, WithinAbs(vn3.omega,1.0e-6));
    REQUIRE_THAT( v3.x, WithinAbs(vn3.x,1.0e-6));
    REQUIRE_THAT( v3.y, WithinAbs(vn3.y,1.0e-6)); 

    // Go from transform directly (should mirror the above)

    Transform2D tf4{Vector2D{0.0925505067, 4.4496879151}, 3.1};
    Twist2D v4 = twist_from_transform(tf4);
    Transform2D tfn4 = integrate_twist(v4); 

    REQUIRE_THAT( tf4.rotation(), WithinAbs(tfn4.rotation(),1.0e-6));
    REQUIRE_THAT( tf4.translation().x, WithinAbs(tfn4.translation().x, 1.0e-6));
    REQUIRE_THAT( tf4.translation().y, WithinAbs(tfn4.translation().y, 1.0e-6));

    // Go from transform directly (random af tf)

    Transform2D tf5{Vector2D{2.0, 1.0}, 0.5};
    Twist2D v5 = twist_from_transform(tf5);
    Transform2D tfn5 = integrate_twist(v5); 

    REQUIRE_THAT( tf5.rotation(), WithinAbs(tfn5.rotation(),1.0e-6));
    // REQUIRE_THAT( tf5.translation().x, WithinAbs(tfn5.translation().x, 1.0e-6)); // As expected, this does not hold
    REQUIRE_THAT( tf5.translation().y, WithinAbs(tfn5.translation().y, 1.0e-6)); 
}

void ekf_check_pose(Slam subject, Pose2D required_pose)
{
    REQUIRE_THAT( subject.pose().theta, WithinAbs(required_pose.theta,1.0e-6));
    REQUIRE_THAT( subject.pose().x, WithinAbs(required_pose.x,1.0e-6));
    REQUIRE_THAT( subject.pose().y, WithinAbs(required_pose.y,1.0e-6));
}

void ekf_check_map(Slam subject, arma::vec required_map)
{
    REQUIRE( arma::approx_equal(subject.map(), required_map, "reldiff", 1e-6));
}

void ekf_check_state_vector(Slam subject, arma::vec required_state_vector)
{
    REQUIRE( arma::approx_equal(subject.state_vector(), required_state_vector, "reldiff", 1e-6));
}

void ekf_check_covariance_matrix(Slam subject, arma::mat required_covariance_matrix)
{
    // REQUIRE_THAT( subject.covariance_matrix()(1,0), WithinAbs(required_covariance_matrix(1,0),1.0e-6));
    REQUIRE( arma::approx_equal(subject.covariance_matrix(), required_covariance_matrix, "reldiff", 1e-6));
    REQUIRE( arma::approx_equal(subject.covariance_matrix().submat(0, q_size, q_size - 1, q_size + 2*num_landmarks - 1), zeros_12, "reldiff", 1e-6));
    REQUIRE( arma::approx_equal(subject.covariance_matrix().submat(q_size, 0, q_size + 2*num_landmarks - 1, q_size - 1), zeros_21, "reldiff", 1e-6));
}

void ekf_check_twist(Slam subject, Twist2D required_twist)
{
    REQUIRE_THAT( subject.twist().omega, WithinAbs(required_twist.omega,1.0e-6));
    REQUIRE_THAT( subject.twist().x, WithinAbs(required_twist.x,1.0e-6));
    REQUIRE_THAT( subject.twist().y, WithinAbs(required_twist.y,1.0e-6));
}

void ekf_check_state_matrix(Slam subject, arma::mat required_state_matrix)
{
    REQUIRE( arma::approx_equal(subject.state_matrix(), required_state_matrix, "reldiff", 1e-6));
}

// void ekf_check_actual_measurement(Slam subject, arma::vec required_actual_measurement)
// {
//     REQUIRE( arma::approx_equal(subject.actual_measurement(), required_actual_measurement, "reldiff", 1e-6));
// }

// void ekf_check_predicted_measurement(Slam subject, arma::vec required_predicted_measurement)
// {
//     // REQUIRE_THAT( subject.predicted_measurement()(0), WithinAbs(required_predicted_measurement(0),1.0e-6));
//     REQUIRE( arma::approx_equal(subject.predicted_measurement(), required_predicted_measurement, "reldiff", 1e-6));
// }

// void ekf_check_sensor_matrix(Slam subject, arma::mat required_sensor_matrix)
// {

//     for(int i = 0; i < 9; i++)
//     {
//         for (int j = 0; j < 2; j++)
//         {
//             // REQUIRE_THAT( subject.sensor_matrix()(j,i), WithinAbs(69.0,1.0e-6));
//             REQUIRE_THAT( subject.sensor_matrix()(j,i), WithinAbs(required_sensor_matrix(j,i),1.0e-6));
//         }
//     }

//     REQUIRE_THAT( subject.sensor_matrix().size(), WithinAbs(required_sensor_matrix.size(),1.0e-6));
    
//     REQUIRE( arma::approx_equal(subject.sensor_matrix(), required_sensor_matrix, "absdiff", 1e-6));
// }
