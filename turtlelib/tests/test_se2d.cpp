#include <sstream>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

using turtlelib::normalize_angle;
using turtlelib::deg2rad;
using turtlelib::rad2deg;
using turtlelib::Point2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;
using turtlelib::Transform2D;
using turtlelib::PI;
using Catch::Matchers::WithinAbs;

TEST_CASE( "2D Twist", "[operator<<]") 
{
    Twist2D tw{0.1, 0.2, 0.3};
    std::string str = "[0.1 0.2 0.3]";

    std::stringstream sstr;
    sstr << tw;
    REQUIRE(sstr.str() == str);
}

TEST_CASE( "2D Twist", "[operator>>]")
{
    Twist2D tw_1{}, tw_2{};
    std::stringstream sstr_1, sstr_2;

    sstr_1 << "0.3 0.2 0.1";
    sstr_1 >> tw_1;

    sstr_2 << "[0.9 0.8 0.7]";
    sstr_2 >> tw_2;

    REQUIRE_THAT( tw_1.omega, WithinAbs(0.3,1.0e-6));
    REQUIRE_THAT( tw_1.x, WithinAbs(0.2,1.0e-6));
    REQUIRE_THAT( tw_1.y, WithinAbs(0.1,1.0e-6));
    REQUIRE_THAT( tw_2.omega, WithinAbs(0.9,1.0e-6));
    REQUIRE_THAT( tw_2.x, WithinAbs(0.8,1.0e-6));
    REQUIRE_THAT( tw_2.y, WithinAbs(0.7,1.0e-6));
}

TEST_CASE( "Initialization of SE(2) identity transform", "[Transform2d()]") 
{
    Transform2D tf;

    REQUIRE_THAT( tf.rotation(), WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( tf.translation().x, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( tf.translation().y, WithinAbs(0.0,1.0e-6));    
}

TEST_CASE( "Pure translation transform", "[Transform2d(Vector2D)]") 
{
    Vector2D displacement{1.0, 1.0};
    Transform2D tf{displacement};

    REQUIRE_THAT( tf.rotation(), WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( tf.translation().x, WithinAbs(1.0,1.0e-6));
    REQUIRE_THAT( tf.translation().y, WithinAbs(1.0,1.0e-6));    
}

TEST_CASE( "Pure rotation transform", "[Transform2d(double)]") 
{
    double angle{4.2*PI};
    Transform2D tf{angle};

    REQUIRE_THAT( tf.rotation(), WithinAbs(0.2*PI,1.0e-6));
    REQUIRE_THAT( tf.translation().x, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( tf.translation().y, WithinAbs(0.0,1.0e-6));    
}

TEST_CASE( "General SE(2) transform", "[Transform2d(Vector2D, double)]") 
{
    Vector2D displacement{1.2, 3.4};
    double angle{4.2*PI};
    Transform2D tf{displacement, angle};

    REQUIRE_THAT( tf.rotation(), WithinAbs(0.2*PI,1.0e-6));
    REQUIRE_THAT( tf.translation().x, WithinAbs(1.2,1.0e-6));
    REQUIRE_THAT( tf.translation().y, WithinAbs(3.4,1.0e-6));    
}

TEST_CASE( "SE(2) transformation of a 2D point", "[Transform2d() Point2D]") 
{
    Vector2D displacement{1.5, 1.5};
    double angle{1.5};

    Transform2D tf{displacement, angle};

    Point2D p{2.5, 2.5};

    Point2D newp = tf(p);

    REQUIRE_THAT( newp.x, WithinAbs(-0.8168944623,1.0e-6));
    REQUIRE_THAT( newp.y, WithinAbs(4.1705804707,1.0e-6));    
}

TEST_CASE( "SE(2) transformation of a 2D vector", "[Transform2d() Vector2D]") 
{
    Vector2D displacement{2.0, 2.0};
    double angle{2.5};

    Transform2D tf{displacement, angle};

    Vector2D v{1.0, 2.2};

    Vector2D newv = tf(v);

    REQUIRE_THAT( newv.x, WithinAbs(-2.1177823326,1.0e-6));
    REQUIRE_THAT( newv.y, WithinAbs(-1.1640438101,1.0e-6));    
}

TEST_CASE( "SE(2) transformation of a 2D twist", "[Transform2d() Twist2D]") 
{
    Vector2D displacement{10.5, 5.1};
    double angle{4.5};

    Transform2D tf{displacement, angle};

    Twist2D v{1.1, 2.2, 3.3};

    Twist2D newv = tf(v);

    REQUIRE_THAT( newv.omega, WithinAbs(1.1,1.0e-6));
    REQUIRE_THAT( newv.x, WithinAbs(8.3720986295,1.0e-6));
    REQUIRE_THAT( newv.y, WithinAbs(-14.396192397,1.0e-6));    
}

TEST_CASE( "Inverse of SE(2) transformation", "[inv()]") 
{
    Vector2D displacement{1.2, 3.4};
    double angle{1.99*PI};
    Transform2D tf{displacement, angle};
    
    Transform2D tf_inv = tf.inv();

    REQUIRE_THAT( tf_inv.rotation(), WithinAbs(0.01*PI,1.0e-6));
    REQUIRE_THAT( tf_inv.translation().x, WithinAbs(-1.0926112916,1.0e-6));
    REQUIRE_THAT( tf_inv.translation().y, WithinAbs(-3.4360152161,1.0e-6));  
}

TEST_CASE( "SE(2) composition operator", "[oprator *=]") 
{
    Vector2D displacement_1{2.5, 5.2};
    double angle_1{4.5*PI};
    Transform2D tf_1{displacement_1, angle_1};

    Vector2D displacement_2{5.2, 2.5};
    double angle_2{5.4*PI};
    Transform2D tf_2{displacement_2, angle_2};
    
    tf_1 *= tf_2;

    REQUIRE_THAT( tf_1.rotation(), WithinAbs(-0.1*PI,1.0e-6));
    REQUIRE_THAT( tf_1.translation().x, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( tf_1.translation().y, WithinAbs(10.4,1.0e-6));  
}

TEST_CASE( "SE(2) transform", "[operator<<]") 
{
    Vector2D displacement{0.5, 0.6};
    double angle{45};
    Transform2D tf{displacement, angle};

    std::stringstream lsstr, rsstr;

    lsstr << tf;
    rsstr << "deg: " << rad2deg(normalize_angle(angle)) << " x: 0.5 y: 0.6";

    REQUIRE(lsstr.str() == rsstr.str());
}

TEST_CASE( "SE(2) transform", "[operator>>]")
{
    Transform2D tf_1{}, tf_2{};
    std::stringstream sstr_a, sstr_b;

    sstr_a << "370.1 0.2 0.3";
    sstr_a >> tf_1;

    sstr_b << "deg: 45.9 x: 0.0 y: 1.0";
    sstr_b >> tf_2;

    REQUIRE_THAT( tf_1.rotation(), WithinAbs(turtlelib::normalize_angle(turtlelib::deg2rad(370.1)), 1.0e-6 ));
    REQUIRE_THAT( tf_1.translation().x, WithinAbs(0.2, 1.0e-6));
    REQUIRE_THAT( tf_1.translation().y, WithinAbs(0.3, 1.0e-6));
    REQUIRE_THAT( tf_2.rotation(), WithinAbs(turtlelib::normalize_angle(turtlelib::deg2rad(45.9)), 1.0e-6));
    REQUIRE_THAT( tf_2.translation().x, WithinAbs(0.0, 1.0e-6) );
    REQUIRE_THAT( tf_2.translation().y, WithinAbs(1.0, 1.0e-6) );
}

TEST_CASE( "SE(2) multiplication operator", "[oprator *]") 
{
    Vector2D displacement_1{5.0, 5.5};
    double angle_1{1.5*PI};
    Transform2D tf_1{displacement_1, angle_1};

    Vector2D displacement_2{2.5, 20.5};
    double angle_2{-5.5*PI};
    Transform2D tf_2{displacement_2, angle_2};
    
    Transform2D tf_3{(tf_1 * tf_2).translation(), (tf_1 * tf_2).rotation()};

    REQUIRE_THAT( tf_3.rotation(), WithinAbs(0*PI,1.0e-6));
    REQUIRE_THAT( tf_3.translation().x, WithinAbs(25.5,1.0e-6));
    REQUIRE_THAT( tf_3.translation().y, WithinAbs(3.0,1.0e-6));  
}
