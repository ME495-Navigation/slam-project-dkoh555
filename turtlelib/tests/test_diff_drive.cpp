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
        DiffDrive robot0{0.69, 0.420, Transform2D{Vector2D{69, 420}, 69}, WheelPosition{4.20, 6.9}};
        WheelPosition delta_phi0{0.0, 0.0};

        robot0.forward_k(delta_phi0);

        REQUIRE_THAT(robot0.get_wheels().right, WithinAbs(normalize_angle(4.20), TOLERANCE));
        REQUIRE_THAT(robot0.get_wheels().left, WithinAbs(normalize_angle(6.9), TOLERANCE));
        REQUIRE_THAT(robot0.get_position().rotation(), WithinAbs(normalize_angle(69), TOLERANCE));
        REQUIRE_THAT(robot0.get_position().translation().x, WithinAbs(69.0, TOLERANCE));
        REQUIRE_THAT(robot0.get_position().translation().y, WithinAbs(420.0, TOLERANCE));
        
        // Linear motion
        DiffDrive turtle1{0.69, 0.420, Transform2D{Vector2D{.69, .420}, .69}, WheelPosition{4.20, 6.9}};
        WheelPosition delta_phi1 = {3.69, 3.69};

        turtle1.forward_k(delta_phi1);

        REQUIRE_THAT(turtle1.get_wheels().right, WithinAbs(normalize_angle(4.20 + 3.69),TOLERANCE));
        REQUIRE_THAT(turtle1.get_wheels().left, WithinAbs(normalize_angle(6.9 + 3.69),TOLERANCE));
        REQUIRE_THAT(turtle1.get_position().rotation(), WithinAbs(normalize_angle(.69),TOLERANCE));
        REQUIRE_THAT(turtle1.get_position().translation().x, WithinAbs(.690 + 0.69 * 3.69 * cos(.690),TOLERANCE)); // CLOSE but still noticable (likely small mistake)
        REQUIRE_THAT(turtle1.get_position().translation().y, WithinAbs(.4200 + 0.69 * 3.69 * sin(.690),TOLERANCE));
        
        // Spinning in place
        DiffDrive robot2{69.0, 420.0, Transform2D{Vector2D{6.9, 0.69}, 69}, WheelPosition{4.20, 6.9}};
        WheelPosition delta_phi2 = {-0.69, 0.69};

        robot2.forward_k(delta_phi2);

        REQUIRE_THAT(robot2.get_wheels().right, WithinAbs(normalize_angle(4.20 - 0.69),TOLERANCE));
        REQUIRE_THAT(robot2.get_wheels().left, WithinAbs(normalize_angle(6.9 + 0.69),TOLERANCE));
        REQUIRE_THAT(robot2.get_position().rotation(), WithinAbs(normalize_angle(69 - 2 * 0.69 * 69.0 / 420.0 ),TOLERANCE));
        REQUIRE_THAT(robot2.get_position().translation().x, WithinAbs(6.9,TOLERANCE));
        REQUIRE_THAT(robot2.get_position().translation().y, WithinAbs(0.69,TOLERANCE));

        // TEST ALLENS
        DiffDrive turtlebot1{33e-3, 160e-3};
        WheelPosition delta_phi_allen = {PI, PI};
        turtlebot1.forward_k(delta_phi_allen);

        REQUIRE_THAT(turtlebot1.get_wheels().right, WithinAbs(PI, TOLERANCE));
        REQUIRE_THAT(turtlebot1.get_wheels().left, WithinAbs(PI, TOLERANCE));
        REQUIRE_THAT(turtlebot1.get_position().rotation(), WithinAbs(0.0, TOLERANCE));
        REQUIRE_THAT(turtlebot1.get_position().translation().x, WithinAbs(PI * 33e-3, TOLERANCE));
        REQUIRE_THAT(turtlebot1.get_position().translation().y, WithinAbs(0.0, TOLERANCE));

        DiffDrive turtlebot2{33e-3, 160e-3};
        WheelPosition delta_phi_allen2 = {2.0 * PI, PI};
        turtlebot2.forward_k(delta_phi_allen2);

        REQUIRE_THAT(turtlebot2.get_wheels().right, WithinAbs(normalize_angle(2 * PI), TOLERANCE));
        REQUIRE_THAT(turtlebot2.get_wheels().left, WithinAbs(normalize_angle(PI), TOLERANCE));

        REQUIRE_THAT(turtlebot2.get_position().rotation(), WithinAbs(normalize_angle(0.6480), 1e-4));
        REQUIRE_THAT(turtlebot2.get_position().translation().x, WithinAbs(0.1449, 1e-4));
        REQUIRE_THAT(turtlebot2.get_position().translation().y, WithinAbs(0.0486, 1e-4));

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