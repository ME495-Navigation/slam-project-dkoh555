#include "turtlelib/geometry2d.hpp"
#include <cmath>
#include <catch2/catch_all.hpp>
#define TOLERANCE 1.0e-6

namespace turtlelib {
    
    TEST_CASE("almost_equal", "geometry2d") {
        double val1 = 1.0;
        double val2 = val1 + 1.0e-13;
        REQUIRE(almost_equal(val1, val2, 1.0e-12) == true);
        val2 += 10.0;
        REQUIRE(almost_equal(val1, val2, 1.0e-12) == false);
        val1 -= val1;
        val2 -= val2;
        REQUIRE(almost_equal(val1, val2, 1.0e-12) == true);
    }

    TEST_CASE("deg2rad", "geometry2d") {
        /// Test case conversions found via https://www.rapidtables.com/convert/number/degrees-to-radians.html
        // 180 deg -> PI rad
        double t_deg = 180.0;
        double t_rad = PI;
        REQUIRE_THAT(deg2rad(t_deg), Catch::Matchers::WithinAbs(t_rad, TOLERANCE));
        // 1 deg -> 0.0055555556*PI rad
        t_deg = 1.0;
        t_rad = 0.0055555556 * PI;
        REQUIRE_THAT(deg2rad(t_deg), Catch::Matchers::WithinAbs(t_rad, TOLERANCE));
        // 0 deg -> 0 rad
        t_deg = 0.0;
        t_rad = 0.0;
        REQUIRE_THAT(deg2rad(t_deg), Catch::Matchers::WithinAbs(t_rad, TOLERANCE));
    }

    TEST_CASE("rad2deg", "geometry2d") {
        /// Test case conversions found via https://www.rapidtables.com/convert/number/radians-to-degrees.html
        // PI rad -> 180 deg
        double t_rad = PI;
        double t_deg = 180.0;
        REQUIRE_THAT(rad2deg(t_rad), Catch::Matchers::WithinAbs(t_deg, TOLERANCE));
        // 30 rad -> 1718.873385 deg
        t_rad = 30;
        t_deg = 1718.873385;
        REQUIRE_THAT(rad2deg(t_rad), Catch::Matchers::WithinAbs(t_deg, TOLERANCE));
        // 0 deg -> 0 rad
        t_rad = 0.0;
        t_deg = 0.0;
        REQUIRE_THAT(rad2deg(t_rad), Catch::Matchers::WithinAbs(t_deg, TOLERANCE));
    }

    TEST_CASE("normalize_angle", "geometry2d") {
        /// Test case conversions found via https://www.rapidtables.com/convert/number/radians-to-degrees.html
        // 3 * PI rad -> PI rad normalized
        double t_degree = 3 * PI;
        double t_normalized = PI;
        REQUIRE_THAT(normalize_angle(t_degree), Catch::Matchers::WithinAbs(t_normalized, TOLERANCE));
        // - PI rad -> 0 rad normalized
        t_degree = - PI;
        t_normalized = PI;
        REQUIRE_THAT(normalize_angle(t_degree), Catch::Matchers::WithinAbs(t_normalized, TOLERANCE));
        // 5 * PI/2 rad -> PI/2 rad normalized
        t_degree = 5 * PI/2;
        t_normalized = PI/2;
        REQUIRE_THAT(normalize_angle(t_degree), Catch::Matchers::WithinAbs(t_normalized, TOLERANCE));
    }

}