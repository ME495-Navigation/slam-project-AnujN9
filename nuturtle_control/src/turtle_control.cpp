/// \file
/// \brief The turtle_control node is responsible for controlling the turtlebot
///
/// PARAMETERS:
///     \brief All these are loaded from diff_params.yaml
///     \param wheel_radius (double): The radius of the wheels [m]
///     \param track_width (double): The distance between the wheels [m]
///     \param motor_cmd_max (double): Maximum motor command value
///     \param motor_cmd_per_rad_sec (double): Motor command per rad/s conversion factor
///     \param encoder_ticks_per_rad (double): Encoder ticks per 1 radian turn
///     \param collision_radius (double): Robot collision radius [m]
///
/// PUBLISHES:
///     \param /wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): Wheel_cmd to move the turtlebot
///     \param /joint_states (sensor_msgs::msg::JointState): angle and velocity to move robot
///
/// SUBSCRIBES:
///     \param /cmd_vel (geometry_msgs::msg::Twist): Converts to wheel_cmd values
///     \param /sensor_data (nuturtlebot_msgs::msg::SensorData): Converts to joint state values
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
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"

using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
  /// \brief Main class of TurtleControl that controls the turtlebot. Publishes the wheel
  ///        commands and joint_states
  ///
  /// \param wheel_radius (double): The radius of the wheels [m]
  /// \param track_width (double): The distance between the wheels [m]
  /// \param motor_cmd_max (double): Maximum motor command value
  /// \param motor_cmd_per_rad_sec (double): Motor command per rad/s conversion factor
  /// \param encoder_ticks_per_rad (double): Encoder ticks per 1 radian turn
  /// \param collision_radius (double): Robot collision radius [m]

public:
  TurtleControl()
  : Node("TurtleControl")
  {
    // Declare parameters and their values
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{}; // For parameter descriptions

    // #### Begin_Citation 1
    param_desc.description = "The wheel radius width in meters";
    declare_parameter("wheel_radius", rclcpp::PARAMETER_DOUBLE, param_desc);
    param_desc.description = "The track width between wheels in meters";
    declare_parameter("track_width", rclcpp::PARAMETER_DOUBLE, param_desc);
    param_desc.description = "Motor command value per rad/s conversion factor";
    declare_parameter("motor_cmd_per_rad_sec", rclcpp::PARAMETER_DOUBLE, param_desc);
    param_desc.description = "Maximum motor command value";
    declare_parameter("motor_cmd_max", rclcpp::PARAMETER_DOUBLE, param_desc);
    param_desc.description = "Motor encoder ticks per radian";
    declare_parameter("encoder_ticks_per_rad", rclcpp::PARAMETER_DOUBLE, param_desc);
    param_desc.description = "The collision radius of the turtlebot";
    declare_parameter("collision_radius", rclcpp::PARAMETER_DOUBLE, param_desc);

    check_parameters();
    // ##### End_Citation 1

    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    motor_cmd_per_rad_sec_ = get_parameter(
      "motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    motor_cmd_max_ = get_parameter(
      "motor_cmd_max").get_parameter_value().get<double>();
    encoder_ticks_per_rad_ = get_parameter(
      "encoder_ticks_per_rad").get_parameter_value().get<double>();
    collision_radius_ = get_parameter("collision_radius").get_parameter_value().get<double>();

    // Create the diff_drive turtlebot
    turtle_ = turtlelib::DiffDrive(wheel_radius_, track_width_);

    // Subscriber
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));
    sensor_data_sub_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(
        &TurtleControl::sensor_callback, this, std::placeholders::_1));

    // Publisher
    wheel_cmd_pub_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", 10);
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);
  }

private:
  // Variables
  double wheel_radius_, track_width_, motor_cmd_per_rad_sec_, motor_cmd_max_,
    encoder_ticks_per_rad_, collision_radius_;
  bool first_data_ = true;
  turtlelib::DiffDrive turtle_;
  turtlelib::Twist2D twist_{0.0, 0.0, 0.0};
  turtlelib::WheelVelocities wheel_vel_{0.0, 0.0};
  nuturtlebot_msgs::msg::WheelCommands wheel_cmd_;
  sensor_msgs::msg::JointState joint_state_, previous_js_;

  // Create objects
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub_;

  /// \brief Checking parameters to see if they are passed by diff_params.yaml
  void check_parameters()
  {
    // List of parameters to be checked
    const std::vector paramList = {"wheel_radius", "track_width", "motor_cmd_per_rad_sec",
      "motor_cmd_max", "encoder_ticks_per_rad", "collision_radius"};
    long unsigned i = 0;
    try {
      for (i = 0; i < paramList.size(); i++) {
        get_parameter(paramList.at(i));
      }
    } catch (rclcpp::exceptions::ParameterUninitializedException &) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Parameter(" << paramList.at(
          i) << ") has not been initialized");
      throw std::invalid_argument("Parameters not declared");
    }
  }

  /// \brief Command Velocity callback which publishes it to wheel command
  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    twist_.omega = msg.angular.z;
    twist_.x = msg.linear.x;
    twist_.y = msg.linear.y;
    // Perform IK on the Twist2D body_twist we extract from geometry_msgs/Twist
    wheel_vel_ = turtle_.InverseKinematics(twist_);
    // Convert the velocity in rad/sec to eoncoder ticks
    wheel_cmd_.left_velocity = static_cast<int>(wheel_vel_.left / motor_cmd_per_rad_sec_);
    wheel_cmd_.right_velocity = static_cast<int>(wheel_vel_.right / motor_cmd_per_rad_sec_);
    // Check for max wheel command speed then publish wheel command
    if (wheel_cmd_.left_velocity > motor_cmd_max_) {
      wheel_cmd_.left_velocity = static_cast<int>(motor_cmd_max_);
    } else if (wheel_cmd_.left_velocity < -motor_cmd_max_) {
      wheel_cmd_.left_velocity = static_cast<int>(-motor_cmd_max_);
    }
    if (wheel_cmd_.right_velocity > motor_cmd_max_) {
      wheel_cmd_.right_velocity = static_cast<int>(motor_cmd_max_);
    } else if (wheel_cmd_.right_velocity < -motor_cmd_max_) {
      wheel_cmd_.right_velocity = static_cast<int>(-motor_cmd_max_);
    }
    wheel_cmd_pub_->publish(wheel_cmd_);
  }

  /// \brief Sensor Data callback which publishes sensor data to joint state publisher
  void sensor_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    joint_state_.header.stamp = msg.stamp;
    joint_state_.name = {"wheel_left_joint", "wheel_right_joint"};
    if (!first_data_) {
      // Converting encoder ticks to angle in radians
      joint_state_.position = {msg.left_encoder / encoder_ticks_per_rad_,
        msg.right_encoder / encoder_ticks_per_rad_};
      // Converting positions to velocity
      double time = (joint_state_.header.stamp.sec - previous_js_.header.stamp.sec) +
        (joint_state_.header.stamp.nanosec - previous_js_.header.stamp.nanosec) * 1e-9;
      joint_state_.velocity = {joint_state_.position.at(0) - previous_js_.position.at(0) / time,
        joint_state_.position.at(0) - previous_js_.position.at(0) / time};
    } else {
      // For the first encoder value
      joint_state_.position = {msg.left_encoder / encoder_ticks_per_rad_,
        msg.right_encoder / encoder_ticks_per_rad_};
      joint_state_.velocity = {0.0, 0.0};
      first_data_ = false;
    }
    previous_js_ = joint_state_;
    joint_state_pub_->publish(joint_state_);
  }
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
