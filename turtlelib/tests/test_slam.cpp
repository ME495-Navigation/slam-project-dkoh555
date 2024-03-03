#include "turtlelib/se2d.hpp"
#include <cmath>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#define TOLERANCE 1.0e-6

using Catch::Matchers::WithinAbs;

namespace turtlelib {
    
    TEST_CASE("initial_armadillo_test", "slam") {
        // Standard test
        Twist2D t_twist{1.2, 4.1, 3.9};
        std::stringstream t_os;
        t_os << t_twist;
        REQUIRE((t_os).str() == "[1.2 4.1 3.9]");

        t_os.str("");
        t_os.clear();
        // Large values
        t_twist.omega = 3.4;
        t_twist.x = 380.2;
        t_twist.y= 0.00345;
        t_os << t_twist;
        REQUIRE((t_os).str() == "[3.4 380.2 0.00345]");

        t_os.str("");
        t_os.clear();
        // Negative values
        t_twist.omega = -3.4;
        t_twist.x = -0.2;
        t_twist.y= -10.8;
        t_os << t_twist;
        REQUIRE((t_os).str() == "[-3.4 -0.2 -10.8]");
    }
}