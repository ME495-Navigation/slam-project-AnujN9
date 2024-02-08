/// \file
/// \brief The nusim node is a simulation of the turtlebot3 robots inrviz.
///
/// PARAMETERS:
///     \param rate (int): Timer callback frequency [Hz]
///     \param x0_ (double): Initial x coordinate of the robot [m]
///     \param y0_ (double): Initial y coordinate of the robot [m]
///     \param theta0_ (double): Initial theta angle of the robot [radians]
///     \param arena_x_length_ (double): Length of arena in x direction [m]
///     \param arena_y_length_ (double): Length of arena in y direction [m]
///     \param obstacles_x_ (std::vector<double>): Vector of x coordinates for each obstacle [m]
///     \param obstacles_y_ (std::vector<double>): Vector of y coordinates for each obstacle [m]
///     \param obstacles_r_ (double): Radius of cylindrical obstacles [m]
///
/// PUBLISHES:
///     \param ~/timestep (std_msgs::msg::UInt64): Simulation timestep
///     \param ~/obstacles (visualization_msgs::msg::MarkerArray): Marker obstacles of cylinders
///     \param ~/walls (visualization_msgs::msg::MarkerArray): Marker of the walls
///
/// SUBSCRIBES:
///     None
///
/// SERVERS:
///     \param ~/reset (std_srvs::srv::Empty): Resets simulation to initial state
///     \param ~/teleport (nusim::srv::Teleport): Teleports robot to a specific pose
///
/// CLIENTS:
///     None
///
/// BROADCASTERS:
///     \param tf_broadcaster_ (tf2_ros::TransformBroadcaster): Broadcasts the robot

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/types.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
  /// \brief Main class of that publishes the timestep and obstacles. Provides the reset and
  ///        teleport services that move the robot around based on the intial pose and pose
  ///        specified respectively.
  ///
  /// \param rate (int): Timer callback frequency [Hz]
  /// \param x0_ (double): Initial x coordinate of the robot [m]
  /// \param y0_ (double): Initial y coordinate of the robot [m]
  /// \param theta0_ (double): Initial theta angle of the robot [radians]
  /// \param arena_x_length_ (double): Length of arena in x direction [m]
  /// \param arena_y_length_ (double): Length of arena in y direction [m]
  /// \param obstacles_x_ (std::vector<double>): Vector of x coordinates for each obstacle [m]
  /// \param obstacles_y_ (std::vector<double>): Vector of y coordinates for each obstacle [m]
  /// \param obstacles_r_ (double): Radius of cylindrical obstacles [m]

public:
  Nusim()
  : Node("Nusim"), timestep_(0), qos_profile(10)
  {
    // Declare parameters and their values
    declare_parameter("rate", 200);   // Hz for timer_callback
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);

    declare_parameter("arena_x_length", 0.0);
    declare_parameter("arena_y_length", 0.0);
    declare_parameter("obstacles_x", std::vector<double>{});
    declare_parameter("obstacles_y", std::vector<double>{});
    declare_parameter("obstacles_r", 0.0);

    // Get params - Read params from yaml file that is passed in the launch file
    int rate = get_parameter("rate").as_int();
    x0_ = get_parameter("x0").as_double();
    y0_ = get_parameter("y0").as_double();
    theta0_ = get_parameter("theta0").as_double();
    arena_x_length_ = get_parameter("arena_x_length").as_double();
    arena_y_length_ = get_parameter("arena_y_length").as_double();
    obstacles_x_ = get_parameter("obstacles_x").as_double_array();
    obstacles_y_ = get_parameter("obstacles_y").as_double_array();
    obstacles_r_ = get_parameter("obstacles_r").as_double();

    xi_ = x0_;
    yi_ = y0_;
    thetai_ = theta0_;

    if (obstacles_x_.size() == obstacles_y_.size()) {
      // this is a valid input
      RCLCPP_INFO_STREAM(get_logger(), "Valid Marker Input!");
      obs_n = obstacles_x_.size();
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "Marker Inputs not the same size!");
      obs_n = 0;
    }

    // Publisher
    timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    rclcpp::QoS qos_profile(10);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    walls_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", qos_profile);
    obstacles_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", qos_profile);

    // Services
    reset_server_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
    teleport_server_ = create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate),
      std::bind(&Nusim::timer_callback, this));

    publish_obstacles();
    publish_walls();
  }

private:
  /// \brief Reset the simulation
  /// \param Request: empty
  /// \param Responce: empty
  void reset_callback(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    /// \param Request empty request
    RCLCPP_INFO_STREAM(get_logger(), "Reset");
    timestep_ = 0;
    x0_ = xi_;
    y0_ = yi_;
    theta0_ = thetai_;
  }

  /// \brief Teleport the robot to a specified pose
  /// \param req: The x,y,theta of position to be teleported to
  /// \param Responce: empty
  void teleport_callback(
    std::shared_ptr<nusim::srv::Teleport::Request> req,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    x0_ = req->x;
    y0_ = req->y;
    theta0_ = req->theta;
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Teleporting to " << x0_ << ", " << y0_ << ", " << theta0_);
  }

  /// \brief Broadcast the TF frames of the robot
  void broadcast_turtle()
  {
    geometry_msgs::msg::TransformStamped t_;

    t_.header.stamp = this->get_clock()->now();
    t_.header.frame_id = "nusim/world";
    t_.child_frame_id = "red/base_footprint";
    t_.transform.translation.x = x0_;
    t_.transform.translation.y = y0_;
    t_.transform.translation.z = 0.0;
    tf2::Quaternion q_;
    q_.setRPY(0, 0, theta0_);
    t_.transform.rotation.x = q_.x();
    t_.transform.rotation.y = q_.y();
    t_.transform.rotation.z = q_.z();
    t_.transform.rotation.w = q_.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t_);
  }

  /// \brief Create walls of the arena and publishes them to a topic
  void publish_walls()
  {
    visualization_msgs::msg::Marker wall_1_, wall_2_, wall_3_, wall_4_;
    wall_1_.header.frame_id = "nusim/world";
    wall_1_.header.stamp = this->get_clock()->now();
    wall_1_.id = 0;
    wall_1_.type = visualization_msgs::msg::Marker::CUBE;
    wall_1_.action = visualization_msgs::msg::Marker::ADD;
    wall_1_.pose.position.x = (arena_x_length_ / 2) + 0.05;
    wall_1_.pose.position.y = 0.0;
    wall_1_.pose.position.z = 0.0;
    wall_1_.pose.orientation.x = 0.0;
    wall_1_.pose.orientation.y = 0.0;
    wall_1_.pose.orientation.z = 0.0;
    wall_1_.pose.orientation.w = 1.0;
    wall_1_.scale.x = 0.1;
    wall_1_.scale.y = arena_y_length_;
    wall_1_.scale.z = 0.25;
    wall_1_.color.r = 1.0f;
    wall_1_.color.g = 0.0f;
    wall_1_.color.b = 0.0f;
    wall_1_.color.a = 1.0;
    walls_.markers.push_back(wall_1_);
    wall_2_.header.frame_id = "nusim/world";
    wall_2_.header.stamp = this->get_clock()->now();
    wall_2_.id = 1;
    wall_2_.type = visualization_msgs::msg::Marker::CUBE;
    wall_2_.action = visualization_msgs::msg::Marker::ADD;
    wall_2_.pose.position.x = 0.0;
    wall_2_.pose.position.y = (arena_y_length_ / 2) + 0.05;
    wall_2_.pose.position.z = 0.0;
    wall_2_.pose.orientation.x = 0.0;
    wall_2_.pose.orientation.y = 0.0;
    wall_2_.pose.orientation.z = 0.0;
    wall_2_.pose.orientation.w = 1.0;
    wall_2_.scale.x = arena_x_length_;
    wall_2_.scale.y = 0.1;
    wall_2_.scale.z = 0.25;
    wall_2_.color.r = 1.0f;
    wall_2_.color.g = 0.0f;
    wall_2_.color.b = 0.0f;
    wall_2_.color.a = 1.0;
    walls_.markers.push_back(wall_2_);
    wall_3_.header.frame_id = "nusim/world";
    wall_3_.header.stamp = this->get_clock()->now();
    wall_3_.id = 2;
    wall_3_.type = visualization_msgs::msg::Marker::CUBE;
    wall_3_.action = visualization_msgs::msg::Marker::ADD;
    wall_3_.pose.position.x = -((arena_x_length_ / 2) + 0.05);
    wall_3_.pose.position.y = 0.0;
    wall_3_.pose.position.z = 0.0;
    wall_3_.pose.orientation.x = 0.0;
    wall_3_.pose.orientation.y = 0.0;
    wall_3_.pose.orientation.z = 0.0;
    wall_3_.pose.orientation.w = 1.0;
    wall_3_.scale.x = 0.1;
    wall_3_.scale.y = arena_y_length_;
    wall_3_.scale.z = 0.25;
    wall_3_.color.r = 1.0f;
    wall_3_.color.g = 0.0f;
    wall_3_.color.b = 0.0f;
    wall_3_.color.a = 1.0;
    walls_.markers.push_back(wall_3_);
    wall_4_.header.frame_id = "nusim/world";
    wall_4_.header.stamp = this->get_clock()->now();
    wall_4_.id = 3;
    wall_4_.type = visualization_msgs::msg::Marker::CUBE;
    wall_4_.action = visualization_msgs::msg::Marker::ADD;
    wall_4_.pose.position.x = 0.0;
    wall_4_.pose.position.y = -((arena_y_length_ / 2) + 0.05);
    wall_4_.pose.position.z = 0.0;
    wall_4_.pose.orientation.x = 0.0;
    wall_4_.pose.orientation.y = 0.0;
    wall_4_.pose.orientation.z = 0.0;
    wall_4_.pose.orientation.w = 1.0;
    wall_4_.scale.x = arena_x_length_;
    wall_4_.scale.y = 0.1;
    wall_4_.scale.z = 0.25;
    wall_4_.color.r = 1.0f;
    wall_4_.color.g = 0.0f;
    wall_4_.color.b = 0.0f;
    wall_4_.color.a = 1.0;
    walls_.markers.push_back(wall_4_);
    walls_publisher_->publish(walls_);
  }

  /// \brief Create obstacles as a MarkerArray and publish them to a topic to display them
  void publish_obstacles()
  {
    for (int i = 0; i < obs_n; i++) {
      visualization_msgs::msg::Marker obstacle_;
      obstacle_.header.frame_id = "nusim/world";
      obstacle_.header.stamp = get_clock()->now();
      obstacle_.id = i;
      obstacle_.type = visualization_msgs::msg::Marker::CYLINDER;
      obstacle_.action = visualization_msgs::msg::Marker::ADD;
      obstacle_.pose.position.x = obstacles_x_.at(i);
      obstacle_.pose.position.y = obstacles_y_.at(i);
      obstacle_.pose.position.z = 0.0;
      obstacle_.pose.orientation.x = 0.0;
      obstacle_.pose.orientation.y = 0.0;
      obstacle_.pose.orientation.z = 0.0;
      obstacle_.pose.orientation.w = 1.0;
      obstacle_.scale.x = obstacles_r_ * 2.0; // Diameter in x
      obstacle_.scale.y = obstacles_r_ * 2.0; // Diameter in y
      obstacle_.scale.z = 0.25;       // Height
      obstacle_.color.r = 1.0f;
      obstacle_.color.g = 0.0f;
      obstacle_.color.b = 0.0f;
      obstacle_.color.a = 1.0;
      obstacles_.markers.push_back(obstacle_);
    }
    obstacles_publisher_->publish(obstacles_);
  }

  /// \brief Main simulation timer loop that publishes markers, tf and timestep
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_;
    //   RCLCPP_INFO_STREAM(get_logger(), "Timestep: " << message.data);
    timestep_publisher_->publish(message);
    broadcast_turtle();
    timestep_++;
  }

  // Variables
  size_t timestep_;
  double x0_, y0_, theta0_;
  double xi_, yi_, thetai_;
  double arena_x_length_, arena_y_length_;
  double obstacles_r_;
  int obs_n;
  std::vector<double> obstacles_x_, obstacles_y_;
  visualization_msgs::msg::MarkerArray obstacles_, walls_;

  // Create objects
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::QoS qos_profile;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_server_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

/// \brief Main function
int main(int argc, char * argv[])
{
  /// \brief Spinning the node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
