/// \file
/// \brief The turtle_control node is responsible for controlling the turtlebot
///
/// PARAMETERS:
///     \brief All these are loaded from diff_params.yaml
///     \param wheel_radius (float): The radius of the wheels [m]
///     \param track_width (float): The distance between the wheels [m]
///     \param motor_cmd_max (float): Maximum motor command value
///     \param motor_cmd_per_rad_sec (float): Motor command per rad/s conversion factor
///     \param encoder_ticks_per_rad (float): Encoder ticks per 1 radian turn
///     \param collision_radius (float): Robot collision radius [m]
///
/// PUBLISHES:
///     None
///
/// SUBSCRIBES:
///     None
///
/// SERVERS:
///     None
///
/// CLIENTS:
///     None

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
  /// \brief Main class of that publishes the timestep and obstacles. Provides the reset and
  ///        teleport services that move the robot around based on the intial pose and pose
  ///        specified respectively.
  ///
  /// \param wheel_radius (float): The radius of the wheels [m]
  /// \param track_width (float): The distance between the wheels [m]
  /// \param motor_cmd_max (float): Maximum motor command value
  /// \param motor_cmd_per_rad_sec (float): Motor command per rad/s conversion factor
  /// \param encoder_ticks_per_rad (float): Encoder ticks per 1 radian turn
  /// \param collision_radius (float): Robot collision radius [m]

public:
  TurtleControl()
  : Node("TurtleControl")
  {
    // Declare parameters and their values
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{}; // For parameter descriptions

    param_desc.description = "The wheel radius width in meters";
    declare_parameter("wheel_radius", 0.0, param_desc);
    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();

    param_desc.description = "The track width between wheels in meters";
    declare_parameter("track_width", 0.0, param_desc);
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();

    param_desc.description = "Motor command value per rad/s conversion factor";
    declare_parameter("motor_cmd_per_rad_sec", 0.0, param_desc);
    motor_cmd_per_rad_sec_ = get_parameter(
      "motor_cmd_per_rad_sec").get_parameter_value().get<double>();

    param_desc.description = "Maximum motor command value";
    declare_parameter("motor_cmd_max", 0.0, param_desc);
    motor_cmd_max_ = get_parameter(
      "motor_cmd_max").get_parameter_value().get<double>();

    param_desc.description = "Motor encoder ticks per radian";
    declare_parameter("encoder_ticks_per_rad", 0.0, param_desc);
    encoder_ticks_per_rad_ = get_parameter(
      "encoder_ticks_per_rad").get_parameter_value().get<double>();

    param_desc.description = "The collision radius of the turtlebot";
    declare_parameter("collision_radius", 0.0, param_desc);
    collision_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();

    if (wheel_radius_ == 0.0 || track_width_ == 0.0 || motor_cmd_per_rad_sec_ == 0.0
    || motor_cmd_max_ == 0.0 || encoder_ticks_per_rad_ == 0.0 || collision_radius_ == 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Parameters are missing in .yaml file");
      throw std::invalid_argument
    }

    // Publisher

    // Services

    // Initialize the transform broadcaster

    // Timer

  }

private:
  /// \brief Reset the simulation
  /// \param Request: empty
  /// \param Responce: empty

  // Variables
  double wheel_radius_, track_width_;
  double motor_cmd_per_rad_sec_, motor_cmd_max_;
  double encoder_ticks_per_rad_, collision_radius_;

  // Create objects
};

/// \brief Main function
int main(int argc, char * argv[])
{
  /// \brief Spinning the node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
