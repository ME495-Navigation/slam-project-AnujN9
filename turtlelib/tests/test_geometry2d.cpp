#include <sstream>
#include <iostream>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "turtlelib/geometry2d.hpp"

using turtlelib::normalize_angle;
using turtlelib::normalizeVector;
using turtlelib::Point2D;
using turtlelib::Vector2D;
using turtlelib::PI;
using Catch::Matchers::WithinAbs;

TEST_CASE( "Normalized angle function", "[normalize_angle]" ) 
{
    // Typecast case
    REQUIRE_THAT( normalize_angle(1), WithinAbs(1.0,1.0e-6));
    // Overflow case
    REQUIRE_THAT(normalize_angle(PI+1), WithinAbs(1.0-PI,1.0e-6));
    // Large Overflow case
    REQUIRE_THAT(normalize_angle(400*PI+1), WithinAbs(1.0,1.0e-6));
    // Upper limit case (included)
    REQUIRE_THAT(normalize_angle(PI), WithinAbs(PI,1.0e-6));
    // Lower limit case (not included)
    REQUIRE_THAT(normalize_angle(-PI), WithinAbs(PI,1.0e-6));
    // Zero case
    REQUIRE_THAT(normalize_angle(0), WithinAbs(0,1.0e-6));
    // Simple case with pi
    REQUIRE_THAT(normalize_angle(-PI/4.0), WithinAbs(-PI/4.0,1.0e-6));
    // Overflow case with pi
    REQUIRE_THAT(normalize_angle(3.0*PI/2.0), WithinAbs(-PI/2.0,1.0e-6));
    // Underflow case with pi
    REQUIRE_THAT(normalize_angle(-5.0*PI/2.0), WithinAbs(-PI/2.0,1.0e-6));
}

TEST_CASE( "2D Point", "[operator<<]") 
{
    Point2D point{4.4, 5.5};
    std::string str = "[4.4 5.5]";
    std::stringstream sstr;
    sstr << point;
    
    REQUIRE(sstr.str() == str);
}

TEST_CASE( "2D Point", "[operator>>]")
{
    Point2D point_1{}, point_2{};
    std::stringstream sstr_1, sstr_2;

    sstr_1 << "1.0 2.0";
    sstr_1 >> point_1;
    sstr_2 << "[3.5 4.5]";
    sstr_2 >> point_2;

    REQUIRE( point_1.x == 1.0 );
    REQUIRE( point_1.y == 2.0 );
    REQUIRE( point_2.x == 3.5 );
    REQUIRE( point_2.y == 4.5 );
}

TEST_CASE( "2D Vector", "[operator<<]") 
{
    Vector2D vec{9.9, 8.84};
    std::string str = "[9.9 8.84]";
    std::stringstream sstr;
    sstr << vec;
    
    REQUIRE(sstr.str() == str);
}

TEST_CASE( "2D Vector", "[operator>>]")
{
    Vector2D vec_1{}, vec_2{};
    std::stringstream sstr_1, sstr_2;
    sstr_1 << "0.1 0.2";
    sstr_1 >> vec_1;
    sstr_2 << "[0.5 2.3]";
    sstr_2 >> vec_2;

    REQUIRE( vec_1.x == 0.1 );
    REQUIRE( vec_1.y == 0.2 );
    REQUIRE( vec_2.x == 0.5 );
    REQUIRE( vec_2.y == 2.3 );
}

TEST_CASE( "Relative vector construction through head and tail points works", "[operator-]") 
{
    REQUIRE_THAT(  (Point2D{1.0, 0.0} - Point2D{4.0, 0.0}).x, WithinAbs(-3.0,1.0e-6));
    REQUIRE_THAT(  (Point2D{0.0, -3.0} - Point2D{0.0, -5.3}).y, WithinAbs(2.3,1.0e-6));
}

TEST_CASE( "Point displacement through relative vector works", "[operator+]") 
{
    REQUIRE_THAT(  (Point2D{-2.6, 0.0} + Vector2D{1.0, -1.0}).x, WithinAbs(-1.6,1.0e-6));
    REQUIRE_THAT(  (Point2D{0.0, 3.0} + Vector2D{1.0, -1.0}).y, WithinAbs(2.0,1.0e-6));
}

TEST_CASE( "Vector normalization works", "[operator+]") // Aditya, Nair
{
    Vector2D v{45, 10.0};

    Vector2D v_hat = normalizeVector(v);

    // Check x.
    REQUIRE_THAT(v_hat.x, WithinAbs(0.9761870602,1.0e-6));
    // Check y.
    REQUIRE_THAT(v_hat.y, WithinAbs(0.2169304578,1.0e-6));
}

// New tests B.8

TEST_CASE("Vector addition", "[operator +]")
{
    Vector2D v1 = {2.0, 2.0};
    Vector2D v2 = {1.0, -3.0};
    Vector2D v3 = v1 + v2;
    
    REQUIRE_THAT(v3.x, WithinAbs(3.0, 1e-5));
    REQUIRE_THAT(v3.y, WithinAbs(-1.0, 1e-5));
}

TEST_CASE("Vector subtraction", "[operator-]")
{
    Vector2D v1 = {2.0, 2.0};
    Vector2D v2 = {1.0, -3.0};
    Vector2D v3 = v1 - v2;
    
    REQUIRE_THAT(v3.x, WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(v3.y, WithinAbs(5.0, 1e-5));
}

TEST_CASE("Vector scaling rhs", "[operator*]")
{
    double scalar = 2.0;
    Vector2D vec = {1.0, -1.0};
    Vector2D s_vec = scalar * vec;
    
    REQUIRE_THAT(s_vec.x, WithinAbs(2.0, 1e-5));
    REQUIRE_THAT(s_vec.y, WithinAbs(-2.0, 1e-5));
}

TEST_CASE("Vector scaling lhs", "[operator*]")
{
    double scalar = 2.0;
    Vector2D vec = {1.0, -1.0};
    Vector2D s_vec = vec * scalar;
    
    REQUIRE_THAT(s_vec.x, WithinAbs(2.0, 1e-5));
    REQUIRE_THAT(s_vec.y, WithinAbs(-2.0, 1e-5));
}

TEST_CASE("Dot product", "[dot]")
{
    Vector2D v1 = {1.0, 2.0};
    Vector2D v2 = {-3.0, -4.5};
    double dot_product = turtlelib::dot(v1, v2);

    REQUIRE_THAT(dot_product, Catch::Matchers::WithinAbs(-12.0, 1e-5));
}

TEST_CASE("Magnitude of the vector", "[magnitude]")
{
    Vector2D v1 = {10.0, -10.0};
    double mag = turtlelib::magnitude(v1);

    REQUIRE_THAT(mag, WithinAbs(14.142135, 1e-5));
}

TEST_CASE("Angle between 2 vectors", "[angle]")
{
    Vector2D v1 = {1, 0};
    Vector2D v2 = {0, 1};
    double ang1 = turtlelib::angle(v1, v2);

    REQUIRE_THAT(ang1, WithinAbs(PI/2, 1e-5));
}
