#include "turtlelib/geometry2d.hpp"
#include <cmath>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

namespace turtlelib {
    TEST_CASE("Almost Equal", "geometry2d") {
        double val1 = 1.0;
        double val2 = val1 + 1.0e-13;
        REQUIRE(almost_equal(val1, val2, 1.0e-12) == true);
    }
}