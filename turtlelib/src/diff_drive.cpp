#include <iostream>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
    DiffDrive::DiffDrive() : wheel_radius(0.033),
                             track_width(0.16), config{0.0, 0.0, 0.0},
                             wheel_position{0.0, 0.0}{}

    DiffDrive::DiffDrive(double radius, double track) : wheel_radius(radius),
                                                                 track_width(track),
                                                                 config{0.0, 0.0, 0.0},
                                                                 wheel_position{0.0, 0.0}{}

    DiffDrive::DiffDrive(double radius, double track, Robot_configuration robot_config) :
                                    wheel_radius(radius),
                                    track_width(track),
                                    config{robot_config},
                                    wheel_position{0.0, 0.0}{}

    Robot_configuration DiffDrive::configuration() const
    {
        return config;
    }

    // ##### Verified with Aditya and Damien, Begin_Citation 1 & 2
    Twist2D DiffDrive::ForwardKinematics(Wheel delta_position)
    {
        wheel_position.left = wheel_position.left + delta_position.left;
        wheel_position.right = wheel_position.right + delta_position.right;

        // The velocitites are the same as delta as it is per unit time
        WheelVelocities wheel_vel;
        wheel_vel.left = delta_position.left;
        wheel_vel.right = delta_position.right;

        // Equation 3
        double body_omega = (wheel_radius/track_width) * (wheel_vel.right - wheel_vel.left);
        double body_x = (wheel_radius/2)*(wheel_vel.left + wheel_vel.right);
        double body_y = 0.0;
        Twist2D body{body_omega,body_x,body_y};

        Transform2D Tb_b_prime = integrate_twist(body); // Transformation matrix between start and end position
        Transform2D Tw_b(Vector2D{config.x, config.y}, config.theta); // Current position to world
        Transform2D Tw_b_prime = Tw_b*Tb_b_prime;
        
        config.x = Tw_b_prime.translation().x; // Updating change in config
        config.y = Tw_b_prime.translation().y;
        config.theta = normalize_angle(Tw_b_prime.rotation());

        return body; // returning twist for odom
    }

    WheelVelocities DiffDrive::InverseKinematics(Twist2D twist)
    {
        WheelVelocities wheel_vel;
        if (twist.y != 0.0)
        {
            throw std::logic_error("Wheel y veloctiy must be 0");
        }
        else
        {
            // do not do (1/X) *, instead just (expr)/X. Use 2.0 instead of 2
            wheel_vel.left = (1/wheel_radius)*(-(track_width/2)*twist.omega + twist.x); // Equation 1
            wheel_vel.right = (1/wheel_radius)*((track_width/2)*twist.omega + twist.x); // Equation 2
        }
        return wheel_vel;
    }
    // ##### Verfied with Aditya and Damien, End_Citation 1 & 2
}
