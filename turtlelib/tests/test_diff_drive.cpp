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
}