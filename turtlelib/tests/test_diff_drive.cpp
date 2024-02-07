#include <sstream>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"

using turtlelib::DiffDrive;
using turtlelib::Wheel;
using turtlelib::WheelVelocities;
using turtlelib::Twist2D;
using turtlelib::PI;
using Catch::Matchers::WithinAbs;

TEST_CASE("Testing for driving forward", "DiffDrive")
{
    DiffDrive turtle(1.0, 1.0);
    // Forward Check
    Wheel new_delta = {PI,PI};
    turtle.ForwardKinematics(new_delta);
    REQUIRE_THAT(turtle.configuration().x, WithinAbs(PI, 1e-5));
    REQUIRE_THAT(turtle.configuration().y, WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(turtle.configuration().theta, WithinAbs(0.0, 1e-5));
    
    // Inverse Check
    Twist2D tw{0, 1, 0};
    WheelVelocities w = turtle.InverseKinematics(tw);
    REQUIRE_THAT(w.left, WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(w.right, WithinAbs(1.0, 1e-5));
    
}

TEST_CASE("Testing for just rotation", "DiffDrive")
{
    DiffDrive turtle(1.0, 1.0);
    // Forward Check
    Wheel new_delta = {-0.5*PI, 0.5*PI};
    turtle.ForwardKinematics(new_delta);
    REQUIRE_THAT(turtle.configuration().x, WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(turtle.configuration().y, WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(turtle.configuration().theta, WithinAbs(PI, 1e-5));
    
    // Inverse Check
    Twist2D tw{PI, 0, 0};
    WheelVelocities w = turtle.InverseKinematics(tw);
    REQUIRE_THAT(w.left, WithinAbs(-0.5*PI, 1e-5));
    REQUIRE_THAT(w.right, WithinAbs(0.5*PI, 1e-5));
    
}

TEST_CASE("Testing for both rotation and translation", "DiffDrive")
{
    DiffDrive turtle(1.0, 1.0);
    // Forward Check
    Wheel new_delta = {0.5*PI, PI};
    turtle.ForwardKinematics(new_delta);
    REQUIRE_THAT(turtle.configuration().x, WithinAbs(1.5, 1e-5));
    REQUIRE_THAT(turtle.configuration().y, WithinAbs(1.5, 1e-5));
    REQUIRE_THAT(turtle.configuration().theta, WithinAbs(PI/2, 1e-5));
    
    // Inverse Check
    DiffDrive turtle2(0.033, 0.16);
    Twist2D tw{PI, 2.0, 0.0};
    WheelVelocities w = turtle2.InverseKinematics(tw);
    REQUIRE_THAT(w.left, WithinAbs(52.9900784155, 1e-5));
    REQUIRE_THAT(w.right, WithinAbs(68.2220427966, 1e-5));
    
}

TEST_CASE("Impossible twist","DiffDrive Inverse")
{
    DiffDrive turtle(1.0, 1.0);
    REQUIRE_THROWS_AS(turtle.InverseKinematics(Twist2D{0.0, 0.0, 1.0}), std::logic_error);
    REQUIRE_THROWS_AS(turtle.InverseKinematics(Twist2D{PI, 10.0, 0.5}), std::logic_error);
}