#include <cstdio>
#include <cmath>
#include <iostream>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    // Print 2D Twist
    std::ostream & operator<<(std::ostream & os, const Twist2D & tw)
    {
        return os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
    }

    // Read 2D Twist
    std::istream & operator>>(std::istream & is, Twist2D & tw)
    {
        const auto c = is.peek();        // examine the next character without extracting it

        if (c == '[') {
            is.get();         // remove the '[' character from the stream
            is >> tw.omega;
            is >> tw.x;
            is >> tw.y;
            is.get();         // remove the ']' character from the stream
        } else {
            is >> tw.omega >> tw.x >> tw.y;
        }

        is.ignore(100, '\n');
        return is;
    }

    // CONSTRUCTORS.

    // Create an identity transformation.
    Transform2D::Transform2D() : 
    tVector{0.0, 0.0}, rotAng{0.0} 
    {}

    // Create a pure translation transform.
    Transform2D::Transform2D(Vector2D displacement) :
    tVector{displacement}
    {}

    // Create a pure rotation transform.
    Transform2D::Transform2D(double angle) :
    rotAng{normalize_angle(angle)}
    {}

    // Create a transform with translation and rotation.
    // First rotate, then translate (in intermediate frame); which is equivalent to first translate, then rotate (in global frame).
    Transform2D::Transform2D(Vector2D displacement, double angle) :
    tVector{displacement}, rotAng{normalize_angle(angle)}
    {}

    // TRANSFORM DIFFERENT ENTITIES THROUGH operator().

    // Transform a point.
    Point2D Transform2D::operator()(Point2D p) const
    {
        Point2D newp{}; // can construct this directly with the args that you are using

        // Rotate.
        newp.x = p.x * cos(rotAng) - p.y * sin(rotAng);
        newp.y = p.x * sin(rotAng) + p.y * cos(rotAng);

        // Translate in global frame.
        newp = newp + tVector;
        
        return newp;
    }

    // Transform a vector.
    Vector2D Transform2D::operator()(Vector2D v) const
    {
        Vector2D newv{};

        // Rotate.
        newv.x = v.x * cos(rotAng) - v.y * sin(rotAng);
        newv.y = v.x * sin(rotAng) + v.y * cos(rotAng);

        return newv;
    }

    // Transform a twist.
    Twist2D Transform2D::operator()(Twist2D v) const
    {
        Twist2D newv{};

        // Multiply with Adjoint.
        newv.omega = v.omega;
        newv.x = v.x * cos(rotAng) - v.y * sin(rotAng) + tVector.y * newv.omega;
        newv.y = v.x * sin(rotAng) + v.y * cos(rotAng) - tVector.x * newv.omega;

        return newv;
    }

    // RETURN INVERSE.
    Transform2D Transform2D::inv() const
    {
        Vector2D newTranslationVector{};
        double newRotationAngle{}; // const auto newRotationAngle = -rotAng

        // R^T
        newRotationAngle = -rotAng;

        // -R^T p
        newTranslationVector.x = -(tVector.x * cos(newRotationAngle) - tVector.y * sin(newRotationAngle));
        newTranslationVector.y = -(tVector.x * sin(newRotationAngle) + tVector.y * cos(newRotationAngle));

        // Create inverted Transform
        Transform2D inv{newTranslationVector, newRotationAngle};

        return inv;
    }

    // COMPOSE TRANSFORMS.
    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {
        // this could be written in a simpler way if you used Transform2D constructors to create
        // temporaries...
        double newRotationAngle{};

        // R_a * R_rhs
        newRotationAngle = normalize_angle(rotAng + rhs.rotation());

        // Convert translation vector to point (p_rhs)
        Point2D temp{rhs.translation().x, rhs.translation().y};

        // R_a * p_rhs + p_a, which is equivalent to T_a * p_rhs
        Vector2D newTranslationVector{(*this)(temp).x, (*this)(temp).y};

        // Rewrite this transform as the composed transform.
        tVector = newTranslationVector;
        rotAng = newRotationAngle;

        return *this;
    }

    // GETTERS.

    // Get translation vector.
    Vector2D Transform2D::translation() const
    {
        return tVector;
    }

    // Get rotation angle.
    double Transform2D::rotation() const
    {
        return rotAng;
    }

    // Print SE(2) Transform.
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
    {
        return os << "deg: " << rad2deg(tf.rotation()) << " x: " << tf.translation().x << " y: " << tf.translation().y;
    }

    // Read SE(2) Transform.
    std::istream & operator>>(std::istream & is, Transform2D & tf) // Adapted from Abhishek Sankar
    {
        std::string trash_1, trash_2, trash_3;
        double rotationAngle {};
        Vector2D translationVector {};
        char next = is.peek(); //look at next character

        if(next == 'd')
        {
            is >> trash_1; //remove deg:
            is >> rotationAngle; //assign rad value to rotation
            is >> trash_2; //remove x:
            is >> translationVector.x; //assign x value to translation.x
            is >> trash_3; //remove y:
            is >> translationVector.y; //assign y value to translation.y
        }
        else
        {
            is >> rotationAngle >> translationVector.x >> translationVector.y;
        }
        tf = Transform2D(translationVector, deg2rad(rotationAngle));
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        lhs *= rhs;

        return lhs;
    }


}
