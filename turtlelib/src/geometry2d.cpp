#include <iostream>
#include <cmath>
#include <cstdio>
#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
    std::ostream & operator<<(std::ostream & os, const Point2D & p)
    {
        return os << "[" << p.x << " " << p.y << "]";
    }

    std::istream & operator>>(std::istream & is, Point2D & p)
    {
        const auto c = is.peek();

        if (c == '[') {
            is.get();
            is >> p.x;
            is >> p.y;
            is.get();
        } else {
            is >> p.x >> p.y;
        }

        is.ignore(100, '\n');
        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        return os << "[" << v.x << " " << v.y << "]";
    }

    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        const auto c = is.peek();

        if (c == '[') {
            is.get();
            is >> v.x;
            is >> v.y;
            is.get();
        } else {
            is >> v.x >> v.y;
        }

        is.ignore(100, '\n');
        return is;
    }

    double normalize_angle(double rad)
    {
        // Normalize to range (-PI, PI].
        if (rad == -PI)
        {
            return PI;
        }
        return atan2(sin(rad), cos(rad));
    }

    Vector2D operator-(const Point2D & head, const Point2D & tail)
    {   
        return Vector2D{head.x - tail.x, head.y - tail.y};
    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp)
    {   
        return Point2D{tail.x + disp.x, tail.y + disp.y};
    }

    Vector2D normalizeVector(const Vector2D & v)
    {
        Vector2D v_hat{};
        double v_norm = sqrt(v.x * v.x + v.y * v.y);

        if(v_norm == 0.0)
        {
            std::cout << "INVALID VECTOR." << std::endl;
        }
        else
        {
            v_hat.x = v.x / v_norm;
            v_hat.y = v.y / v_norm;
        }
        return v_hat;
    }

    Vector2D & Vector2D::operator+=(const Vector2D & rhs)
    {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator-=(const Vector2D & rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator*=(const double & rhs)
    {
        x *= rhs;
        y *= rhs;
        return *this;
    }

    Vector2D operator+(Vector2D lhs, const Vector2D & rhs)
    {
        return lhs+=rhs;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D & rhs)
    {
        return lhs-=rhs;
    }

    Vector2D operator*(Vector2D lhs, const double & rhs)
    {
        return lhs*=rhs;
    }

    Vector2D operator*(const double & lhs, Vector2D rhs)
    {
        return rhs*=lhs;
    }

    double dot(Vector2D vec1, Vector2D vec2)
    {
        return vec1.x*vec2.x + vec1.y*vec2.y;
    }

    double magnitude(Vector2D vec)
    {
        return sqrt(vec.x*vec.x + vec.y*vec.y);
    }

    double angle(Vector2D vec1, Vector2D vec2)
    {
        return atan2(vec1.x*vec2.y-vec1.y*vec2.x, vec1.x*vec2.x+vec1.y*vec2.y);
    }
}
