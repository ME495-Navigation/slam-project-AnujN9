/// \file
/// \brief The odometry node handles the odometry values of the turtlebot
///
/// PARAMETERS:
///     \param wheel_radius (double): The radius of the wheels [m]
///     \param track_width (double): The distance between the wheels [m]
///     \param body_id (std::string): The name of the body frame of the robot
///     \param odom_id (std::string): The name of the odometry frame
///     \param wheel_left (std::string): The name of the left wheel joint
///     \param wheel_right (std::string): The name of the right wheel joint
///
/// PUBLISHES:
///     \param /odom (nav_msgs::msg::Odometry): Publishes odometry state
///
/// SUBSCRIBES:
///     \param /joint_states (sensor_msgs::msg:JointState): Recieves joint states to update odom
///
/// SERVERS:
///     \param /initial_pose (nuturtle_control::srv::InitialPose): Reset the config of turtlebot
///
/// CLIENTS:
///     None
///
/// BROADCASTERS:
///     \param tf_broadcaster_ (tf2_ros::TransformBroadcaster): Broadcasts the turtle position

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nuturtle_control/srv/initial_pose.hpp"

using namespace std::chrono_literals;

class Odometry : public rclcpp::Node
{
  /// \brief Main class that deals with the Odometry of the turtlebot. Provides a service
  ///        that gives the configuration of the robot.
  ///
  /// \param wheel_radius (double): The radius of the wheels [m]
  /// \param track_width (double): The distance between the wheels [m]
  /// \param body_id (std::string): The name of the body frame of the robot
  /// \param odom_id (std::string): The name of the odometry frame
  /// \param wheel_left (std::string): The name of the left wheel joint
  /// \param wheel_right (std::string): The name of the right wheel joint

public:
  Odometry()
  : Node("Odometry")
  {
    // Declare parameters and their values
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{}; // For parameter descriptions
    param_desc.description = "The wheel radius width in meters";
    declare_parameter("wheel_radius", rclcpp::PARAMETER_DOUBLE, param_desc);
    param_desc.description = "The track width between wheels in meters";
    declare_parameter("track_width", rclcpp::PARAMETER_DOUBLE, param_desc);
    param_desc.description = "The name of the body frame of the robot";
    declare_parameter("body_id", rclcpp::PARAMETER_STRING, param_desc);
    param_desc.description = "The name of the odometry frame";
    declare_parameter("odom_id", "odom", param_desc);
    param_desc.description = "The name of the left wheel joint";
    declare_parameter("wheel_left", rclcpp::PARAMETER_STRING, param_desc);
    param_desc.description = "The name of the right wheel joint";
    declare_parameter("wheel_right", rclcpp::PARAMETER_STRING, param_desc);

    check_parameters();

    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    body_id_ = get_parameter("body_id").get_parameter_value().get<std::string>();
    odom_id_ = get_parameter("odom_id").get_parameter_value().get<std::string>();
    wheel_left_ = get_parameter("wheel_left").get_parameter_value().get<std::string>();
    wheel_right_ = get_parameter("wheel_right").get_parameter_value().get<std::string>();

    // Subscriber
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&Odometry::joint_state_callback, this, std::placeholders::_1));

    // Publisher
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    //Service
    initial_pose_srv_ = create_service<nuturtle_control::srv::InitialPose>(
      "initial_pose",
      std::bind(
        &Odometry::initial_pose_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //Create the turtlebot
    turtlebot_ = turtlelib::DiffDrive {wheel_radius_, track_width_};

    // Setting the odom message and tf_broadcasted message
    nav_odom_msg_.header.frame_id = odom_id_;
    nav_odom_msg_.child_frame_id = body_id_;
    nav_odom_msg_.pose.pose.position.z = 0.0;
    geo_odom_tf_.header.frame_id = odom_id_;
    geo_odom_tf_.child_frame_id = body_id_;
    geo_odom_tf_.transform.translation.z = 0.0;
  }

private:
  // Variables
  std::string body_id_, odom_id_, wheel_left_, wheel_right_;
  double wheel_radius_, track_width_;
  turtlelib::DiffDrive turtlebot_{wheel_radius_, track_width_};
  turtlelib::Wheel pre_pos_{0.0, 0.0};
  turtlelib::Twist2D body_twist_;

  nav_msgs::msg::Odometry nav_odom_msg_;
  geometry_msgs::msg::TransformStamped geo_odom_tf_;
  tf2::Quaternion quaternion_;

  // Create objects
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /// \brief Checking parameters to see if they are being passed
  void check_parameters()
  {
    // List of parameters to be checked
    const std::vector paramList = {"wheel_radius", "track_width", "body_id",
      "wheel_left", "wheel_right"};
    long unsigned i = 0;
    try {
      for (i = 0; i < paramList.size(); i++) {
        get_parameter(paramList.at(i));
      }
    } catch (rclcpp::exceptions::ParameterUninitializedException &) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Parameter(" << paramList.at(i) << ") has not been initialized");
      throw std::invalid_argument("Parameters not declared");
    }
  }

  /// \brief Joint States callback which updates the robot odometry and publishes it
  void joint_state_callback(const sensor_msgs::msg::JointState & msg)
  {
    // Getting the twist and updating the robot config
    body_twist_ = turtlebot_.ForwardKinematics(
      {msg.position.at(0) - pre_pos_.left, msg.position.at(1) - pre_pos_.right});

    // Updating and publishing the odometry
    nav_odom_msg_.header.stamp = get_clock()->now();
    nav_odom_msg_.pose.pose.position.x = turtlebot_.configuration().x;
    nav_odom_msg_.pose.pose.position.y = turtlebot_.configuration().y;
    quaternion_.setRPY(0, 0, turtlebot_.configuration().theta);
    nav_odom_msg_.pose.pose.orientation.x = quaternion_.x();
    nav_odom_msg_.pose.pose.orientation.y = quaternion_.y();
    nav_odom_msg_.pose.pose.orientation.z = quaternion_.z();
    nav_odom_msg_.pose.pose.orientation.w = quaternion_.w();
    nav_odom_msg_.twist.twist.linear.x = body_twist_.x;
    nav_odom_msg_.twist.twist.linear.y = body_twist_.y;
    nav_odom_msg_.twist.twist.angular.z = body_twist_.omega;
    odom_pub_->publish(nav_odom_msg_);

    // Broadcasting the transform
    geo_odom_tf_.header.stamp = get_clock()->now();
    geo_odom_tf_.transform.translation.x = turtlebot_.configuration().x;
    geo_odom_tf_.transform.translation.y = turtlebot_.configuration().y;
    geo_odom_tf_.transform.rotation.x = quaternion_.x();
    geo_odom_tf_.transform.rotation.y = quaternion_.y();
    geo_odom_tf_.transform.rotation.z = quaternion_.z();
    geo_odom_tf_.transform.rotation.w = quaternion_.w();
    tf_broadcaster_->sendTransform(geo_odom_tf_);

    // Update the previous wheel position
    pre_pos_.left = msg.position[0];
    pre_pos_.right = msg.position[1];
  }

  /// \brief Resets the robot configuration
  /// \param req: The x, y, theta of robot position to reset to
  /// \param Responce: empty
  void initial_pose_callback(
    std::shared_ptr<nuturtle_control::srv::InitialPose::Request> req,
    std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
  {
    turtlebot_ = turtlelib::DiffDrive{wheel_radius_, track_width_, {req->x, req->y, \
        req->theta}};
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Reset to " << turtlebot_.configuration().x << ", " << turtlebot_.configuration().y << ", " <<
        turtlebot_.configuration().theta);
  }
};

/// \brief Main function
int main(int argc, char * argv[])
{
  /// \brief Spinning the node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
