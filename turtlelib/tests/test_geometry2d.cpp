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
        // PI rad -> PI rad normalized
        double t_degree = PI;
        double t_normalized = PI;
        REQUIRE_THAT(normalize_angle(t_degree), Catch::Matchers::WithinAbs(t_normalized, TOLERANCE));
        // - PI rad -> PI rad normalized
        t_degree = - PI;
        t_normalized = - PI;
        REQUIRE_THAT(normalize_angle(t_degree), Catch::Matchers::WithinAbs(t_normalized, TOLERANCE));
        // - PI/4 rad -> - PI/4 rad normalized
        t_degree = - PI/4;
        t_normalized = - PI/4;
        REQUIRE_THAT(normalize_angle(t_degree), Catch::Matchers::WithinAbs(t_normalized, TOLERANCE));
        // 0 rad -> 0 rad normalized
        t_degree = 0;
        t_normalized = 0;
        REQUIRE_THAT(normalize_angle(t_degree), Catch::Matchers::WithinAbs(t_normalized, TOLERANCE));
        // 3 * PI/2 rad -> - PI/2 rad normalized
        t_degree = 3 * PI/2;
        t_normalized = - PI/2;
        REQUIRE_THAT(normalize_angle(t_degree), Catch::Matchers::WithinAbs(t_normalized, TOLERANCE));
        // - 5 * PI/2 rad -> PI/2 rad normalized
        t_degree = - 5 * PI/2;
        t_normalized = - PI/2;
        REQUIRE_THAT(normalize_angle(t_degree), Catch::Matchers::WithinAbs(t_normalized, TOLERANCE));
    }

    TEST_CASE("output_point", "geometry2d") {
        // Standard test
        Point2D t_point{4.1, 3.9};
        std::stringstream t_os;
        t_os << t_point;
        REQUIRE((t_os).str() == "[4.1 3.9]");

        t_os.str("");
        t_os.clear();
        // Large values
        t_point.x = 380.2;
        t_point.y= 0.00345;
        t_os << t_point;
        REQUIRE((t_os).str() == "[380.2 0.00345]");

        t_os.str("");
        t_os.clear();
        // Negative values
        t_point.x = -0.2;
        t_point.y= -10.8;
        t_os << t_point;
        REQUIRE((t_os).str() == "[-0.2 -10.8]");
    }

    TEST_CASE("input_point", "geometry2d") {
        Point2D t_point;
        std::stringstream t_is;

        // Standard test
        t_is.str("2.9 10.8");
        t_is >> t_point;
        REQUIRE(((t_point.x == 2.9) && (t_point.y == 10.8)));

        t_is.str("");
        t_is.clear();
        // Input with brackets
        t_is.str("[304.3 50.6]");
        t_is >> t_point;
        REQUIRE(((t_point.x == 304.3) && (t_point.y == 50.6)));

        t_is.str("");
        t_is.clear();
        // Negative values
        t_is.str("-1.2 -3.4056");
        t_is >> t_point;
        REQUIRE(((t_point.x == -1.2) && (t_point.y == -3.4056)));
    }

    TEST_CASE("subtract_points", "geometry2d") {
        // Standard test
        REQUIRE_THAT((Point2D{3.4, 0.0} - Point2D{3.4, 5.5}).x, Catch::Matchers::WithinAbs((Vector2D{0.0, -5.5}).x, TOLERANCE));
        REQUIRE_THAT((Point2D{3.4, 0.0} - Point2D{3.4, 5.5}).y, Catch::Matchers::WithinAbs(Vector2D{0.0, -5.5}.y, TOLERANCE));

        // Standard test no.2
        REQUIRE_THAT((Point2D{18.4, 9.3} - Point2D{-4.4, 2.1}).y, Catch::Matchers::WithinAbs((Vector2D{22.8, 7.2}).y, TOLERANCE));
    }

    TEST_CASE("add_point_and_vector", "geometry2d") {
        // Standard test
        REQUIRE_THAT((Point2D{3.4, 0.0} + Vector2D{3.4, 5.5}).x, Catch::Matchers::WithinAbs((Point2D{6.8, 5.5}).x, TOLERANCE));
        REQUIRE_THAT((Point2D{3.4, 0.0} + Vector2D{3.4, 5.5}).y, Catch::Matchers::WithinAbs(Point2D{6.8, 5.5}.y, TOLERANCE));

        // Standard test no.2
        REQUIRE_THAT((Point2D{18.4, 9.3} + Vector2D{-4.4, 2.1}).y, Catch::Matchers::WithinAbs((Point2D{14.0, 11.4}).y, TOLERANCE));
    }

    TEST_CASE("output_vector", "geometry2d") {
        // Standard test
        Vector2D t_vector{4.1, 3.9};
        std::stringstream t_os;
        t_os << t_vector;
        REQUIRE((t_os).str() == "[4.1 3.9]");

        t_os.str("");
        t_os.clear();
        // Large values
        t_vector.x = 380.2;
        t_vector.y= 0.00345;
        t_os << t_vector;
        REQUIRE((t_os).str() == "[380.2 0.00345]");

        t_os.str("");
        t_os.clear();
        // Negative values
        t_vector.x = -0.2;
        t_vector.y= -10.8;
        t_os << t_vector;
        REQUIRE((t_os).str() == "[-0.2 -10.8]");
    }

    TEST_CASE("input_vector", "geometry2d") {
        Vector2D t_vector;
        std::stringstream t_is;

        // Standard test
        t_is.str("2.9 10.8");
        t_is >> t_vector;
        REQUIRE(((t_vector.x == 2.9) && (t_vector.y == 10.8)));

        t_is.str("");
        t_is.clear();
        // Input with brackets
        t_is.str("[304.3 50.6]");
        t_is >> t_vector;
        REQUIRE(((t_vector.x == 304.3) && (t_vector.y == 50.6)));

        t_is.str("");
        t_is.clear();
        // Negative values
        t_is.str("-1.2 -3.4056");
        t_is >> t_vector;
        REQUIRE(((t_vector.x == -1.2) && (t_vector.y == -3.4056)));
    }
}