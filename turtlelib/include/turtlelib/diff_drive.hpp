#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Kinematics of a differential drive of the turtlebot.

#include<iosfwd>
#include<cmath>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    /// \brief Position of both wheels
    struct Wheel
    {
        /// \brief left wheel position in radians
        double left = 0.0;

        /// \brief right wheel position in radians
        double right = 0.0;
    };

    /// \brief Velocity of both wheels
    struct WheelVelocities
    {
        /// \brief left wheel velocity
        double left = 0.0;

        /// \brief right wheel velocity
        double right = 0.0;
    };

    /// \brief Robot body configuration
    struct Robot_configuration
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief the orientation angle
        double theta = 0.0;
    };

    /// \brief Kinematics of a differential drive robot.
    class DiffDrive
    {
    private:
        /// \brief radius of wheels
        double wheel_radius;
        /// \brief distance between the wheels
        double track_width;
        /// \brief robot configuration
        Robot_configuration config;
        /// \brief position of both wheels
        Wheel wheel_position;

    public:
        /// \brief start at origin and default to turtlebo3 burger specifications
        DiffDrive();

        /// \brief set wheel specifications and start at origin
        /// \param radius - radius of wheels
        /// \param width - width between the wheels
        DiffDrive(double radius, double width);

        /// \brief set start configuration and wheel specifications
        /// \param radius - radius of wheels
        /// \param width - width between the wheels
        /// \param robot_config - robot body configuration
        DiffDrive(double radius, double width, Robot_configuration robot_config);

        /// \brief the robots body configuration
        /// \return robots current body configuration
        Robot_configuration configuration() const;

        /// \brief Calculates the forward kinematics from the new wheel positions
        /// \param delta_positions - difference between wheel positions
        /// \return new robot configuration
        Twist2D ForwardKinematics(Wheel delta_positions);

        /// \brief Calculates the inverse kinematics from a body twist
        /// \param twist - the body twist to preform inverse kinematics on
        /// \return wheel velocities required for the twist
        WheelVelocities InverseKinematics(Twist2D twist);
    };

}

#endif