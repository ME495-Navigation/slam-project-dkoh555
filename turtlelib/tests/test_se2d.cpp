#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <cmath>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#define TOLERANCE 1.0e-6

using Catch::Matchers::WithinAbs;

namespace turtlelib {
    
    TEST_CASE("output_twist", "se2d") {
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

    TEST_CASE("input_twist", "se2d") {
        Twist2D t_twist;
        std::stringstream t_is;

        // Standard test
        t_is.str("3.4 2.9 10.8");
        t_is >> t_twist;
        REQUIRE(((t_twist.omega == 3.4) && (t_twist.x == 2.9) && (t_twist.y == 10.8)));

        t_is.str("");
        t_is.clear();
        // Input with brackets
        t_is.str("[3.4 304.3 50.6]");
        t_is >> t_twist;
        REQUIRE(((t_twist.omega == 3.4) && (t_twist.x == 304.3) && (t_twist.y == 50.6)));

        t_is.str("");
        t_is.clear();
        // Negative values
        t_is.str("-3.4 -1.2 -3.4056");
        t_is >> t_twist;
        REQUIRE(((t_twist.omega == -3.4) && (t_twist.x == -1.2) && (t_twist.y == -3.4056)));
    }

    TEST_CASE("constructors", "se2d") {
        Transform2D vec1;
        REQUIRE(((vec1.translation().x == 0.0) && (vec1.translation().y == 0.0)
            && (vec1.rotation() == 0.0)));
        Transform2D vec2{Vector2D{2.0, 1.9}};
        REQUIRE(((vec2.translation().x == 2.0) && (vec2.translation().y == 1.9)
            && (vec2.rotation() == 0.0)));
        Transform2D vec3{3.14};
        REQUIRE(((vec3.translation().x == 0.0) && (vec3.translation().y == 0.0)
            && (vec3.rotation() == 3.14)));
        Transform2D vec4{Vector2D{2.0, 1.9}, 3.14};
        REQUIRE(((vec4.translation().x == 2.0) && (vec4.translation().y == 1.9)
            && (vec4.rotation() == 3.14)));
    }

    TEST_CASE( "SE(2) transformation of a 2D point works", "se2d") // Aditya Nair
    {
        Vector2D displacement{4.20, 6.9};
        double angle{6.9};

        Transform2D tf{displacement, angle};

        Point2D p{6.9, 4.20};

        Point2D newp = tf(p);

        REQUIRE_THAT( newp.x, WithinAbs(7.399056180434521,1.0e-6));
        REQUIRE_THAT( newp.y, WithinAbs(14.317279794805081,1.0e-6));    
    }

    TEST_CASE( "SE(2) transformation of a 2D vector works", "se2d") // Aditya Nair
    {
        Vector2D displacement{4.20, 6.9};
        double angle{6.9};

        Transform2D tf{displacement, angle};

        Vector2D v{6.9, 4.20};

        Vector2D newv = tf(v);

        REQUIRE_THAT( newv.x, WithinAbs(3.199056180434521,1.0e-6));
        REQUIRE_THAT( newv.y, WithinAbs(7.417279794805081,1.0e-6));    
    }

    TEST_CASE( "SE(2) transformation of a 2D twist works", "se2d") // Aditya Nair
    {
        Vector2D displacement{4.20, 6.9};
        double angle{6.9};

        Transform2D tf{displacement, angle};

        Twist2D v{6.9, 6.9, 4.20};

        Twist2D newv = tf(v);

        REQUIRE_THAT( newv.omega, WithinAbs(6.9,1.0e-6));
        REQUIRE_THAT( newv.x, WithinAbs(50.809056180434524,1.0e-6));
        REQUIRE_THAT( newv.y, WithinAbs(-21.562720205194921,1.0e-6));    
    }

    TEST_CASE("inverse", "se2d") 
    {
        Transform2D trans{Vector2D{3.9, -1.54}, 3.01};
        Transform2D inv_trans = trans.inv();
        REQUIRE_THAT(inv_trans.rotation(), Catch::Matchers::WithinRel(-3.01, 0.001));
        REQUIRE_THAT(inv_trans.translation().x, Catch::Matchers::WithinRel(4.068, 0.001));
        REQUIRE_THAT(inv_trans.translation().y, Catch::Matchers::WithinRel(-1.014, 0.001));
    }

    TEST_CASE("output_transform", "[transform2d]")
    {        
        Transform2D t_trans{Vector2D{1.0, 5.01}, deg2rad(90.0)};
        std::ostringstream t_os;
        t_os << t_trans;
        REQUIRE(t_os.str() == "deg: 90 x: 1 y: 5.01");
    }

    TEST_CASE("input_twist", "[twist2d]")
    {
        Twist2D t_twist;
        std::istringstream t_is;
        t_is.str("[4.9 2.1 9.02]");
        t_is >> t_twist;
        REQUIRE(t_twist.omega == 4.9);
        REQUIRE(t_twist.x == 2.1);
        REQUIRE(t_twist.y == 9.02);
    }


    TEST_CASE("input_transform", "[transform2d]")
    {
        Transform2D t_trans;
        std::istringstream t_is("90.0 -1.345 10.83");
        t_is >> t_trans;
        REQUIRE(t_trans.translation().x == -1.345);
        REQUIRE(t_trans.translation().y == 10.83);
        REQUIRE(t_trans.rotation() == deg2rad(90.0));
    }



    TEST_CASE( "SE(2) composition operator works", "[oprator *=]") // Aditya Nair
    {
        Vector2D displacement_1{4.20, 6.9};
        double angle_1{-6.9*PI};
        Transform2D tf_1{displacement_1, angle_1};

        Vector2D displacement_2{6.9, 4.20};
        double angle_2{4.2*PI};
        Transform2D tf_2{displacement_2, angle_2};
        
        tf_1 *= tf_2;

        REQUIRE_THAT( tf_1.rotation(), WithinAbs(-0.7*PI,1.0e-6));
        REQUIRE_THAT( tf_1.translation().x, WithinAbs(-1.064418586061780,1.0e-6));
        REQUIRE_THAT( tf_1.translation().y, WithinAbs(0.773345370373218,1.0e-6));  
    }

    TEST_CASE("Rotation and Translation", "se2d") // Stella Yu
    {
    double x = 3.0;
    double y = 5.0;
    double ang = 6.2876;
    Transform2D T(Vector2D{x, y}, ang);
    REQUIRE(T.rotation() == ang);
    REQUIRE(T.translation().x == x);
    REQUIRE(T.translation().y == y);
    }

    TEST_CASE("operator()(Vector2D v)", "se2d") // Stella Yu
    {
    double test_rot = PI / 2.0;
    double test_x = 0.0;
    double test_y = 1.0;
    Transform2D T_ab{Vector2D{test_x, test_y}, test_rot};
    Vector2D v_b{1, 1};
    Vector2D v_a = T_ab(v_b);
    REQUIRE_THAT(v_a.x, Catch::Matchers::WithinAbs(-1, 1e-5));
    REQUIRE_THAT(v_a.y, Catch::Matchers::WithinAbs(1, 1e-5));
    }

    TEST_CASE( "SE(2) multiplication operator works", "[oprator *]") // Aditya Nair
    {
        Vector2D displacement_1{4.20, 6.9};
        double angle_1{-6.9*PI};
        Transform2D tf_1{displacement_1, angle_1};

        Vector2D displacement_2{6.9, 4.20};
        double angle_2{4.2*PI};
        Transform2D tf_2{displacement_2, angle_2};
        
        Transform2D tf_3{(tf_1 * tf_2).translation(), (tf_1 * tf_2).rotation()};

        REQUIRE_THAT( tf_3.rotation(), WithinAbs(-0.7*PI,1.0e-6));
        REQUIRE_THAT( tf_3.translation().x, WithinAbs(-1.064418586061780,1.0e-6));
        REQUIRE_THAT( tf_3.translation().y, WithinAbs(0.773345370373218,1.0e-6));  
    }
}