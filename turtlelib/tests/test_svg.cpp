#include <catch2/catch_all.hpp>
#include "turtlelib/svg.hpp"
#include <sstream>

using Catch::Matchers::WithinAbs;

namespace turtlelib
{
    TEST_CASE( "width_getter", "svg") 
    {
        SVG svg(902, 1080);

        REQUIRE(svg.get_width() ==  902);
    }
}