/// \file
/// \brief The circle node commands the turtlebot to move in circles
///
/// PARAMETERS:
///     \param frequency (int): The frequency of the node
///
/// PUBLISHES:
///     \param /cmd_vel (geometry_msgs::msg::Twist): command velocity to move the turtlebot
///
/// SUBSCRIBES:
///     None
///
/// SERVERS:
///     \param /control (nuturtle_control::srv::Control): Service to control the radius
///                     and velocity of cmd_vel
///     \param /reverse (std_srvs::srv::Empty): Service that reverses direction of cmd_vel
///     \param /stop (std_srvs::srv::Empty): Service to stop publishing cmd_vel
///
/// CLIENTS:
///     None
///
/// BROADCASTERS:
///     None

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

class Circle : public rclcpp::Node
{
  /// \brief Main class of Circle that commands the turtlebot to move in a circle.
  ///
  /// \param frequency (int): Frequency at which cmd_vel is published

public:
  Circle()
  : Node("Circle")
  {
    // Declare parameters and their values
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{}; // For parameter descriptions
    param_desc.description = "The frequency at which cmd_vel is published";
    declare_parameter("frequency", 100, param_desc);
    frequency_ = get_parameter("frequency").get_parameter_value().get<int>();

    // Publisher
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Service
    control_srv_ = create_service<nuturtle_control::srv::Control>(
      "control",
      std::bind(
        &Circle::control_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    reverse_srv_ = create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(
        &Circle::reverse_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = create_service<std_srvs::srv::Empty>(
      "stop",
      std::bind(
        &Circle::stop_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    // Timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / frequency_),
      std::bind(&Circle::timer_callback, this));
  }

private:
  // Variables
  int frequency_;
  bool stop_flag_ = true;
  geometry_msgs::msg::Twist cmd_twist_;

  // Create objects
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  /// \brief Control service that moves the robot in a circular motion
  /// \param req: The velocity and radius to move the robot in
  /// \param Responce: empty
  void control_callback(
    std::shared_ptr<nuturtle_control::srv::Control::Request> req,
    std::shared_ptr<nuturtle_control::srv::Control::Response>)
  {
    cmd_twist_.linear.x = req->radius * req->velocity;
    cmd_twist_.angular.z = req->velocity;
    stop_flag_ = false;
  }

  /// \brief Reverse service that moves the robot in the opposite direction
  /// \param Request: empty
  /// \param Responce: empty
  void reverse_callback(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    cmd_twist_.linear.x = -cmd_twist_.linear.x;
    cmd_twist_.angular.z = -cmd_twist_.angular.z;
    stop_flag_ = false;
  }

  /// \brief Stop service that stops moving the robot
  /// \param Request: empty
  /// \param Responce: empty
  void stop_callback(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    cmd_twist_.linear.x = 0.0;
    cmd_twist_.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_twist_);
    stop_flag_ = true;
  }

  /// \brief Timer that publishes the cmd_vel
  void timer_callback()
  {
    if (stop_flag_ == false) {
      cmd_vel_pub_->publish(cmd_twist_);
    }
  }
};

/// \brief Main function
int main(int argc, char * argv[])
{
  /// \brief Spinning the node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
