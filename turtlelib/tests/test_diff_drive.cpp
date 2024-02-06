#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <cmath>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#define TOLERANCE 1.0e-6

using Catch::Matchers::WithinAbs;

namespace turtlelib {
    
    TEST_CASE( "forward_kinematics", "diff_drive") 
    {
        // No motion
        DiffDrive robot0{0.1, 0.5, Transform2D{Vector2D{102, -302}, 0.0}, WheelPosition{1.223, -0.21}};
        WheelPosition wheel_motion0{0.0, 0.0};
        robot0.forward_k(wheel_motion0);

        REQUIRE_THAT(robot0.get_wheels().right, WithinAbs(normalize_angle(1.223), TOLERANCE));
        REQUIRE_THAT(robot0.get_wheels().left, WithinAbs(normalize_angle(-0.21), TOLERANCE));
        REQUIRE_THAT(robot0.get_position().rotation(), WithinAbs(normalize_angle(0.0), TOLERANCE));
        REQUIRE_THAT(robot0.get_position().translation().x, WithinAbs(102.0, TOLERANCE));
        REQUIRE_THAT(robot0.get_position().translation().y, WithinAbs(-302.0, TOLERANCE));
        
        // Pure linear motion
        DiffDrive robot1{0.1, 0.5, Transform2D{Vector2D{18.43, 123}, 101}, WheelPosition{1.223, -0.21}};
        WheelPosition wheel_motion1 = {1.31, 1.31};

        robot1.forward_k(wheel_motion1);

        REQUIRE_THAT(robot1.get_wheels().right, WithinAbs(normalize_angle(1.223 + 1.31),TOLERANCE));
        REQUIRE_THAT(robot1.get_wheels().left, WithinAbs(normalize_angle(-0.21 + 1.31),TOLERANCE));
        REQUIRE_THAT(robot1.get_position().rotation(), WithinAbs(normalize_angle(101),TOLERANCE));
        REQUIRE_THAT(robot1.get_position().translation().x, WithinAbs(18.43 + 0.1 * 1.31 * cos(101),TOLERANCE));
        REQUIRE_THAT(robot1.get_position().translation().y, WithinAbs(123 + 0.1 * 1.31 * sin(101),TOLERANCE));
        
        // Pure rotational motion (spinning in place)
        DiffDrive robot2{11.2, 354.1, Transform2D{Vector2D{-119, -1.02}, 0.0}, WheelPosition{4.20, 6.9}};
        WheelPosition wheel_motion2 = {-1.02, 1.02};

        robot2.forward_k(wheel_motion2);

        REQUIRE_THAT(robot2.get_wheels().right, WithinAbs(normalize_angle(4.20 - 1.02),TOLERANCE));
        REQUIRE_THAT(robot2.get_wheels().left, WithinAbs(normalize_angle(6.9 + 1.02),TOLERANCE));
        REQUIRE_THAT(robot2.get_position().rotation(), WithinAbs(normalize_angle(0.0 - (1.02 + 1.02) * 11.2 / 354.1 ),TOLERANCE));
        REQUIRE_THAT(robot2.get_position().translation().x, WithinAbs(-119,TOLERANCE));
        REQUIRE_THAT(robot2.get_position().translation().y, WithinAbs(-1.02,TOLERANCE));

        // Quarter circular arc
        DiffDrive robot3{0.69, 0.420, Transform2D{Vector2D{69, 420}, 69}, WheelPosition{4.20, 6.9}};
        WheelPosition delta_phi3 = {2 * (0.420 / 0.69) * (PI / 2), (0.420 / 0.69) * (PI / 2)};

        robot3.forward_k(delta_phi3);

        REQUIRE_THAT(robot3.get_wheels().left, WithinAbs(normalize_angle(6.9 + (0.420 / 0.69) * (PI / 2)),TOLERANCE));
        REQUIRE_THAT(robot3.get_wheels().right, WithinAbs(normalize_angle(4.20 + 2 * (0.420 / 0.69) * (PI / 2)),TOLERANCE));
        REQUIRE_THAT(robot3.get_position().rotation(), WithinAbs(normalize_angle(69 + PI/2),TOLERANCE));
        REQUIRE_THAT(robot3.get_position().translation().x, WithinAbs(69 + 0.63 * sqrt(2) * cos(69 + PI/4),TOLERANCE));
        REQUIRE_THAT(robot3.get_position().translation().y, WithinAbs(420 + 0.63 * sqrt(2) * sin(69 + PI/4),TOLERANCE));

        // Random arc
        DiffDrive robot4{69.0, 420.0, Transform2D{Vector2D{6.9, 0.69}, 69}, WheelPosition{4.20, 6.9}};
        WheelPosition delta_phi4 = {-0.420, 0.69};

        robot4.forward_k(delta_phi4);

        REQUIRE_THAT(robot4.get_wheels().left, WithinAbs(normalize_angle(6.9 + 0.69),TOLERANCE));
        REQUIRE_THAT(robot4.get_wheels().right, WithinAbs(normalize_angle(4.20 - 0.420),TOLERANCE));
        REQUIRE_THAT(robot4.get_position().rotation(), WithinAbs(normalize_angle(69 - (0.69 + 0.420) * 69.0/420.0 ),TOLERANCE));
        REQUIRE_THAT(robot4.get_position().translation().x, WithinAbs(16.0050106563,TOLERANCE));
        REQUIRE_THAT(robot4.get_position().translation().y, WithinAbs(-1.2146835484,TOLERANCE));
    }

    TEST_CASE("inverse_kinematics", "diff_drive")
    {
        // Pure linear motion
        DiffDrive robot0{10.2, 112.3};
        WheelPosition wheel_motion0 = robot0.inverse_k(Twist2D{0.0, 3.5, 0.0});

        REQUIRE_THAT(wheel_motion0.left, WithinAbs(3.5 / 10.2, TOLERANCE));
        REQUIRE_THAT(wheel_motion0.right, WithinAbs(3.5 / 10.2, TOLERANCE));

        DiffDrive robot1{10.2, 112.3};
        WheelPosition wheel_motion1 = robot1.inverse_k(Twist2D{0.0, -1034.4, 0.0});

        REQUIRE_THAT(wheel_motion1.right, WithinAbs(-1034.4 / 10.2, TOLERANCE));
        REQUIRE_THAT(wheel_motion1.left, WithinAbs(-1034.4 / 10.2, TOLERANCE));

        // Pure rotation
        DiffDrive robot2{3.2, 12.34};
        WheelPosition wheel_motion2 = robot2.inverse_k(Twist2D{PI, 0.0, 0.0});

        REQUIRE_THAT(wheel_motion2.left, WithinAbs(PI * (12.34 / 2.0) / 3.2, TOLERANCE));
        REQUIRE_THAT(wheel_motion2.right, WithinAbs(-PI * (12.34 / 2.0) / 3.2, TOLERANCE));

        DiffDrive robot3{3.2, 12.34};
        WheelPosition wheel_motion3 = robot3.inverse_k(Twist2D{-PI, 0.0, 0.0});

        REQUIRE_THAT(wheel_motion3.left, WithinAbs(-PI * (12.34 / 2.0) / 3.2, TOLERANCE));
        REQUIRE_THAT(wheel_motion3.right, WithinAbs(PI * (12.34 / 2.0) / 3.2, TOLERANCE));

        // Circular arc
        DiffDrive robot4{0.023, 0.88234};
        WheelPosition wheel_motion4 = robot4.inverse_k(Twist2D{PI, PI * 0.88234 / 2.0, 0.0});

        REQUIRE_THAT(wheel_motion4.right, WithinAbs(0.0, TOLERANCE));
        REQUIRE_THAT(wheel_motion4.left, WithinAbs(PI * 0.88234 / 0.023, TOLERANCE));

        // Random arc
        DiffDrive robot5{0.023, 0.88234};

        REQUIRE_THROWS_AS(robot5.inverse_k(Twist2D{0.0, 0.0, 1.0}), std::logic_error);
        REQUIRE_THROWS_AS(robot5.inverse_k(Twist2D{PI, 10000.0, 0.5}), std::logic_error);
    }
}