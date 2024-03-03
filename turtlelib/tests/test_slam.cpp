#include "turtlelib/slam.hpp"
#include <cmath>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#define TOLERANCE 1.0e-6

using Catch::Matchers::WithinAbs;

namespace turtlelib {
    
    arma::mat test_matrix = arma::mat(2, 2, arma::fill::eye);

    TEST_CASE("initial_armadillo_test", "slam") {
        auto val = test_matrix(0, 0);
        REQUIRE(val == 1);

        val = test_matrix(1,0);
        REQUIRE(val == 0);

        test_matrix.ones();
        val = test_matrix(1,0);
        REQUIRE(val == 1);
    }
}