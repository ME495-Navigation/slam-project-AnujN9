/// \file
/// \brief The nusim node is a simulation of the turtlebot3 robots inrviz.
///
/// PARAMETERS:
///     \param rate (int): Timer callback frequency [Hz]
///     \param x0 (double): Initial x coordinate of the robot [m]
///     \param y0 (double): Initial y coordinate of the robot [m]
///     \param theta0 (double): Initial theta angle of the robot [radians]
///     \param arena_x_length (double): Length of arena in x direction [m]
///     \param arena_y_length (double): Length of arena in y direction [m]
///     \param obstacles_x (std::vector<double>): Vector of x coordinates for each obstacle [m]
///     \param obstacles_y (std::vector<double>): Vector of y coordinates for each obstacle [m]
///     \param obstacles_r (double): Radius of cylindrical obstacles [m]
///     \param wheel_radius (double): The radius of the wheels [m]
///     \param track_width (double): The distance between the wheels [m]
///     \param motor_cmd_max (double): Maximum motor command value
///     \param motor_cmd_per_rad_sec (double): Motor command per rad/s conversion factor
///     \param encoder_ticks_per_rad (double): Encoder ticks per 1 radian turn
///     \param collision_radius (double): Robot collision radius [m]
///     \param draw_only (bool) Only draw obstacles and walls
///     \param input_noise (double): Noise added to input signals from turtlebot
///     \param slip_fraction (double): Wheel slippage factor for turtlebot
///     \param max_range (double): Max sensor laser range
///     \param basic_sensor_variance (double): Laser sensor variance
///     \param lidar_min_range (double): Minimum range of lidar [m]
///     \param lidar_max_range (double): Maximum range of lidar [m]
///     \param lidar_angle_increment (double): Angle increment between lidar samples [rad]
///     \param lidar_sample (double): Number of samples per rotation
///     \param lidar_resolution (double): Resolution of lidar distance
///     \param lidar_noise (double): Noise of lidar sample
///
/// PUBLISHES:
///     \param ~/timestep (std_msgs::msg::UInt64): Simulation timestep
///     \param ~/obstacles (visualization_msgs::msg::MarkerArray): Marker obstacles of cylinders
///     \param ~/walls (visualization_msgs::msg::MarkerArray): Marker of the walls
///     \param red/sensor_data (nuturtlebot_msgs::msg::SensorData): current simulated sensor
///            readings
///     \param red/path (nav_msgs::msg::Path) the path of the robot
///     \param /fake_sensor (visualization_msgs::msg::MarkerArray) the fake sensor data of the robot
///     \param /laser_scan (sensor_msgs::msg::LaserScan) the lidar scan data of the robot
///
/// SUBSCRIBES:
///     \param red/wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): reads twist to update robot
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
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/types.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
  /// \brief Main class of that publishes the timestep and obstacles. Provides the reset and
  ///        teleport services that move the robot around based on the intial pose and pose
  ///        specified respectively.
  ///
  /// \param rate (int): Timer callback frequency [Hz]
  /// \param x0 (double): Initial x coordinate of the robot [m]
  /// \param y0 (double): Initial y coordinate of the robot [m]
  /// \param theta0 (double): Initial theta angle of the robot [radians]
  /// \param arena_x_length (double): Length of arena in x direction [m]
  /// \param arena_y_length (double): Length of arena in y direction [m]
  /// \param obstacles_x (std::vector<double>): Vector of x coordinates for each obstacle [m]
  /// \param obstacles_y (std::vector<double>): Vector of y coordinates for each obstacle [m]
  /// \param obstacles_r (double): Radius of cylindrical obstacles [m]
  /// \param wheel_radius (double): The radius of the wheels [m]
  /// \param track_width (double): The distance between the wheels [m]
  /// \param motor_cmd_max (double): Maximum motor command value
  /// \param motor_cmd_per_rad_sec (double): Motor command per rad/s conversion factor
  /// \param encoder_ticks_per_rad (double): Encoder ticks per 1 radian turn
  /// \param collision_radius (double): Robot collision radius [m]
  /// \param draw_only (bool) Only draw obstacles and walls
  /// \param input_noise (double): Noise added to input signals from turtlebot
  /// \param slip_fraction (double): Wheel slippage factor for turtlebot
  /// \param max_range (double): Max sensor laser range
  /// \param basic_sensor_variance (double): Laser sensor variance
  /// \param lidar_min_range (double): Minimum range of lidar [m]
  /// \param lidar_max_range (double): Maximum range of lidar [m]
  /// \param lidar_angle_increment (double): Angle increment between lidar samples [rad]
  /// \param lidar_sample (double): Number of samples per rotation
  /// \param lidar_resolution (double): Resolution of lidar distance
  /// \param lidar_noise (double): Noise of lidar sample

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

    declare_parameter("draw_only", false);

    declare_parameter("input_noise", 0.0);
    declare_parameter("slip_fraction", 0.0);
    declare_parameter("max_range", 0.5);
    declare_parameter("basic_sensor_variance", 0.01);
    declare_parameter("lidar_min_range", 0.12);
    declare_parameter("lidar_max_range", 3.5);
    declare_parameter("lidar_angle_increment", 0.01745329238474369);
    declare_parameter("lidar_sample", 360.0);
    declare_parameter("lidar_resolution", 1.0);
    declare_parameter("lidar_noise", 0.0);

    // Get params - Read params from yaml file that is passed in the launch file
    rate_ = get_parameter("rate").as_int();
    x0_ = get_parameter("x0").as_double();
    y0_ = get_parameter("y0").as_double();
    theta0_ = get_parameter("theta0").as_double();
    arena_x_length_ = get_parameter("arena_x_length").as_double();
    arena_y_length_ = get_parameter("arena_y_length").as_double();
    obstacles_x_ = get_parameter("obstacles_x").as_double_array();
    obstacles_y_ = get_parameter("obstacles_y").as_double_array();
    obstacles_r_ = get_parameter("obstacles_r").as_double();

    // Declare parameters and their values
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{}; // For parameter descriptions
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

    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    motor_cmd_per_rad_sec_ = get_parameter(
      "motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    motor_cmd_max_ = get_parameter(
      "motor_cmd_max").get_parameter_value().get<double>();
    encoder_ticks_per_rad_ = get_parameter(
      "encoder_ticks_per_rad").get_parameter_value().get<double>();
    collision_radius_ = get_parameter("collision_radius").get_parameter_value().get<double>();

    draw_only_ = get_parameter("draw_only").as_bool();

    input_noise_ = get_parameter("input_noise").as_double();
    slip_fraction_ = get_parameter("slip_fraction").as_double();
    max_range_ = get_parameter("max_range").as_double();
    basic_sensor_variance_ = get_parameter("basic_sensor_variance").as_double();
    lidar_min_range_ = get_parameter("lidar_min_range").as_double();
    lidar_max_range_ = get_parameter("lidar_max_range").as_double();
    lidar_angle_increment_ = get_parameter("lidar_angle_increment").as_double();
    lidar_sample_ = get_parameter("lidar_sample").as_double();
    lidar_resolution_ = get_parameter("lidar_resolution").as_double();
    lidar_noise_ = get_parameter("lidar_noise").as_double();

    xi_ = x0_;
    yi_ = y0_;
    thetai_ = theta0_;

    // turtlelib::Robot_configuration con{x0_,y0_,theta0_};
    turtlebot_ = turtlelib::DiffDrive{wheel_radius_, track_width_};

    if (obstacles_x_.size() == obstacles_y_.size()) {
      // this is a valid input
      RCLCPP_INFO_STREAM(get_logger(), "Valid Marker Input!");
      obs_n = obstacles_x_.size();
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "Marker Inputs not the same size!");
      obs_n = 0;
    }

    // Publisher
    timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("/timestep", 10);
    rclcpp::QoS qos_profile(10);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    rclcpp::QoS scan_qos_profile =
      rclcpp::QoS(rclcpp::KeepLast(20)).durability_volatile().best_effort();

    walls_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/walls", qos_profile);
    obstacles_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/obstacles", qos_profile);

    if (!draw_only_) {
      sensor_data_pub_ = create_publisher<nuturtlebot_msgs::msg::SensorData>(
        "red/sensor_data", 10);
      path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "red/path", 10);
      fake_sensor_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/fake_sensor", qos_profile);
      lidar_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
        "/laser_scan", scan_qos_profile);

      // Subscriber
      wheel_cmd_sub_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
        "red/wheel_cmd", 10,
        std::bind(&Nusim::wheel_cmd_callback, this, std::placeholders::_1));

      // Services
      reset_server_ = create_service<std_srvs::srv::Empty>(
        "~/reset",
        std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
      teleport_server_ = create_service<nusim::srv::Teleport>(
        "~/teleport",
        std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate_),
      std::bind(&Nusim::timer_callback, this));
    lidar_timer_ = create_wall_timer(
      200ms,
      std::bind(&Nusim::lidar_timer_callback, this));

    publish_obstacles();
    publish_walls();
  }

private:
  // Variables
  size_t timestep_;
  int rate_;
  double x0_, y0_, theta0_;
  double xi_, yi_, thetai_;
  double arena_x_length_, arena_y_length_;
  double obstacles_r_;
  double wheel_radius_, track_width_, motor_cmd_per_rad_sec_, motor_cmd_max_,
    encoder_ticks_per_rad_, collision_radius_;
  size_t obs_n;
  double input_noise_, slip_fraction_, max_range_, basic_sensor_variance_, lidar_noise_,
    lidar_min_range_, lidar_max_range_, lidar_angle_increment_, lidar_sample_, lidar_resolution_;
  bool draw_only_ = false;
  std::vector<double> obstacles_x_, obstacles_y_;
  visualization_msgs::msg::MarkerArray obstacles_, walls_;
  turtlelib::DiffDrive turtlebot_;
  turtlelib::WheelVelocities new_wheel_vel_{0.0, 0.0};
  turtlelib::Wheel wheel_pos_{0.0, 0.0}, abs_wheel_pos_{0.0, 0.0};
  turtlelib::Twist2D body_{0.0, 0.0, 0.0};
  turtlelib::Robot_configuration r_;
  nuturtlebot_msgs::msg::SensorData sensor_data_;
  nav_msgs::msg::Path red_path_;
  sensor_msgs::msg::LaserScan lidar_scan_data_;

  // Create objects
  rclcpp::TimerBase::SharedPtr timer_, lidar_timer_;
  rclcpp::QoS qos_profile;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_server_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub_;

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

  /// \brief Broadcast the TF frames of the robot and the path
  void broadcast_turtle()
  {
    geometry_msgs::msg::TransformStamped t_;
    geometry_msgs::msg::PoseStamped pose;

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

    // Publish the red path
    pose.header.frame_id = "nusim/world";
    pose.header.stamp = get_clock()->now();
    pose.pose.position.x = x0_;
    pose.pose.position.y = y0_;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = q_.x();
    pose.pose.orientation.y = q_.y();
    pose.pose.orientation.z = q_.z();
    pose.pose.orientation.w = q_.w();
    red_path_.header.stamp = get_clock()->now();
    red_path_.header.frame_id = "nusim/world";
    red_path_.poses.push_back(pose);
    path_pub_->publish(red_path_);
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
    for (size_t i = 0; i < obs_n; i++) {
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

  /// \brief Uses wheel command from mcu to rad/sec
  /// \param msg: wheel commands that get converted to wheel velocities
  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
  {
    double noise_l = 0.0, noise_r = 0.0;
    std::normal_distribution<> noise{0.0, input_noise_};
    if (msg.left_velocity != 0) {
      noise_l = noise(get_random());
    }
    if (msg.right_velocity != 0) {
      noise_r = noise(get_random());
    }
    new_wheel_vel_.left = static_cast<double>(msg.left_velocity) * motor_cmd_per_rad_sec_;
    new_wheel_vel_.left += noise_l;
    new_wheel_vel_.right = static_cast<double>(msg.right_velocity) * motor_cmd_per_rad_sec_;
    new_wheel_vel_.right += noise_r;
  }

  /// \brief Updates the robot config based on the wheel_cmd data and updates robot config for tf
  void update_config()
  {
    double slip_l = 0.0, slip_r = 0.0;
    std::uniform_real_distribution<> slip{-slip_fraction_, slip_fraction_};
    slip_l = slip(get_random());
    slip_r = slip(get_random());

    wheel_pos_.left = new_wheel_vel_.left * (1 + slip_l) / rate_;
    wheel_pos_.right = new_wheel_vel_.right * (1 + slip_r) / rate_;
    body_ = turtlebot_.ForwardKinematics(wheel_pos_);

    check_collision();
    r_ = turtlebot_.configuration();
    x0_ = r_.x;
    y0_ = r_.y;
    theta0_ = r_.theta;
  }

  /// \brief Updates and publishes sensor data
  void send_sensor_data()
  {
    abs_wheel_pos_.left += wheel_pos_.left;
    abs_wheel_pos_.right += wheel_pos_.right;
    sensor_data_.stamp = get_clock()->now();
    sensor_data_.left_encoder = static_cast<int>(abs_wheel_pos_.left * encoder_ticks_per_rad_);
    sensor_data_.right_encoder = static_cast<int>(abs_wheel_pos_.right * encoder_ticks_per_rad_);
    sensor_data_pub_->publish(sensor_data_);
  }

  /// \brief Publishes the fake sensor data of obstacles
  void publish_fake_sensor()
  {
    visualization_msgs::msg::MarkerArray fa;
    std::normal_distribution<> o_noise{0.0, basic_sensor_variance_};
    for (size_t i = 0; i < obs_n; i++) {
      visualization_msgs::msg::Marker o;
      o.header.frame_id = "red/base_footprint";
      o.header.stamp = get_clock()->now();
      o.id = i;
      o.type = 3;
      double dx = turtlebot_.configuration().x - obstacles_x_.at(i);
      double dy = turtlebot_.configuration().y - obstacles_y_.at(i);
      double d = std::sqrt(std::pow((dx), 2) + std::pow((dy), 2));
      if (d < max_range_ - collision_radius_ - obstacles_r_) {
        o.action = 0;
        o.color.r = 1.0;
        o.color.g = 1.0;
        o.color.b = 0.0;
        o.color.a = 1.0;
        o.scale.x = 2 * obstacles_r_;
        o.scale.y = 2 * obstacles_r_;
        o.scale.z = 0.25;
        turtlelib::Transform2D Two(turtlelib::Vector2D{obstacles_x_.at(i), obstacles_y_.at(i)}, 0);
        turtlelib::Transform2D Twr(turtlelib::Vector2D{
          turtlebot_.configuration().x, turtlebot_.configuration().y},
          turtlebot_.configuration().theta);
        turtlelib::Vector2D pos = (Twr.inv() * Two).translation();
        auto pos_noise = o_noise(get_random());
        o.pose.position.x = pos.x + pos_noise;
        pos_noise = o_noise(get_random());
        o.pose.position.y = pos.y + pos_noise;
        o.pose.position.z = 0.0;
      } else {
        o.action = 2;
      }
      fa.markers.push_back(o);
    }
    fake_sensor_pub_->publish(fa);
  }

  /// \brief Publishes the laser scan data of the lidar
  // TODO check if this works
  void publish_lidar_scan()
  {
    lidar_scan_data_.header.frame_id = "red/base_scan";
    lidar_scan_data_.header.stamp = get_clock()->now();
    lidar_scan_data_.angle_min = 0.0;
    lidar_scan_data_.angle_max = 2 * turtlelib::PI;
    lidar_scan_data_.angle_increment = lidar_angle_increment_;
    lidar_scan_data_.time_increment = 0.0;
    lidar_scan_data_.range_min = lidar_min_range_;
    lidar_scan_data_.range_max = lidar_max_range_;
    lidar_scan_data_.ranges.clear();
    for (int i = 0; i < lidar_sample_; i++) {
      double lidar_val = lidar_max_range_;
      const auto x_max = turtlebot_.configuration().x +
        lidar_max_range_ * cos(i * lidar_angle_increment_ + turtlebot_.configuration().theta);
      const auto y_max = turtlebot_.configuration().y +
        lidar_max_range_ * sin(i * lidar_angle_increment_ + turtlebot_.configuration().theta);
      const auto slope = (y_max - turtlebot_.configuration().y)
        / (x_max - turtlebot_.configuration().x);
      for (size_t j = 0; j < obs_n; j++) {
        const auto val = turtlebot_.configuration().y - slope
          * turtlebot_.configuration().x - obstacles_y_.at(j);
        const auto a = 1.0 + slope * slope;
        const auto b = 2 * (val * slope - obstacles_x_.at(j));
        const auto c = obstacles_x_.at(j) * obstacles_x_.at(j) + val * val -
          obstacles_r_ * obstacles_r_;
        const auto det = b * b - 4 * a * c;
        if (det < 0) { // no solution
          // Checking walls
          const double wall1_x = 0.5 * arena_x_length_;
          const double wall1_y = slope * (wall1_x - turtlebot_.configuration().x)
            + turtlebot_.configuration().y;
          const bool direction_wall1_x = (turtlebot_.configuration().x <= wall1_x)
            && (wall1_x <= x_max);
          const auto distance1 = sqrt(pow((wall1_x - turtlebot_.configuration().x), 2)
            + pow((wall1_y - turtlebot_.configuration().y), 2));
          if ((distance1 < lidar_max_range_) && (distance1 < lidar_val) && direction_wall1_x) {
            lidar_val = distance1;
          }
          const double wall2_x = -0.5 * arena_x_length_;
          const double wall2_y = slope * (wall2_x - turtlebot_.configuration().x)
            + turtlebot_.configuration().y;
          const bool direction_wall2_x = (x_max <= wall2_x)
            && (wall2_x <= turtlebot_.configuration().x);
          const auto distance2 = sqrt(pow((wall2_x - turtlebot_.configuration().x), 2)
            + pow((wall2_y - turtlebot_.configuration().y), 2));
          if ((distance2 < lidar_max_range_) && (distance2 < lidar_val) && direction_wall2_x) {
            lidar_val = distance2;
          }
          const double wall3_y = 0.5 * arena_y_length_;
          const double wall3_x = (wall3_y - turtlebot_.configuration().y)
            / slope + turtlebot_.configuration().x;
          const bool direction_wall3_y = (turtlebot_.configuration().y <= wall3_y)
            && (wall3_y <= y_max);
          const auto distance3 = sqrt(pow((wall3_x - turtlebot_.configuration().x), 2)
            + pow((wall3_y - turtlebot_.configuration().y), 2));
          if ((distance3 < lidar_max_range_) && (distance3 < lidar_val) && direction_wall3_y) {
            lidar_val = distance3;
          }
          const double wall4_y = -0.5 * arena_y_length_;
          const double wall4_x = (wall4_y - turtlebot_.configuration().y)
            / slope + turtlebot_.configuration().x;
          const bool direction_wall4_y = (y_max <= wall4_y)
            && (wall4_y <= turtlebot_.configuration().y);
          const auto distance4 = sqrt(pow((wall4_x - turtlebot_.configuration().x), 2)
            + pow((wall4_y - turtlebot_.configuration().y), 2));
          if ((distance4 < lidar_max_range_) && (distance4 < lidar_val) && direction_wall4_y) {
            lidar_val = distance4;
          }
        } else if (det == 0.0) { // 1 solution
          const double o_x = (-1.0 * b) / (2 * a);
          const double o_y = slope * (o_x - turtlebot_.configuration().x)
            + turtlebot_.configuration().y;
          const auto distance = sqrt(pow((o_x - turtlebot_.configuration().x), 2)
            + pow((o_y - turtlebot_.configuration().y), 2));
          const double directionx = ((o_x - turtlebot_.configuration().x)
            / (x_max - turtlebot_.configuration().x));
          const double directiony = ((o_y - turtlebot_.configuration().y)
            / (y_max - turtlebot_.configuration().y));
          if ((distance < lidar_max_range_) && (distance < lidar_val) &&
            (directionx > 0.0) && (directiony > 0.0))
          {
            lidar_val = distance;
          }
        } else if (det > 0) { // 2 solutions
          const double x1 = (-1.0 * b + sqrt(det)) / (2 * a);
          const double y1 = slope * (x1 - turtlebot_.configuration().x)
            + turtlebot_.configuration().y;
          const auto distance5 = sqrt(pow((x1 - turtlebot_.configuration().x), 2)
            + pow((y1 - turtlebot_.configuration().y), 2));
          const double direction1x = ((x1 - turtlebot_.configuration().x)
            / (x_max - turtlebot_.configuration().x));
          const double direction1y = ((y1 - turtlebot_.configuration().y)
            / (y_max - turtlebot_.configuration().y));
          const double x2 = (-1.0 * b - sqrt(det)) / (2 * a);
          const double y2 = slope * (x2 - turtlebot_.configuration().x)
            + turtlebot_.configuration().y;
          const auto distance6 = sqrt(pow((x2 - turtlebot_.configuration().x), 2)
            + pow((y2 - turtlebot_.configuration().y), 2));
          const double direction2x = ((x2 - turtlebot_.configuration().x)
            / (x_max - turtlebot_.configuration().x));
          const double direction2y = ((y2 - turtlebot_.configuration().y)
            / (y_max - turtlebot_.configuration().y));
          if ((distance5 < lidar_max_range_) && (distance5 < lidar_val) &&
            (direction1x > 0.0) && (direction1y > 0.0))
          {
            lidar_val = distance5;
          }
          if ((distance6 < lidar_max_range_) && (distance6 < lidar_val) &&
            (direction2x > 0.0) && (direction2y > 0.0))
          {
            lidar_val = distance6;
          }
        }
      }
      if (lidar_val != lidar_max_range_) {
        std::normal_distribution<> lnoise{0.0, lidar_noise_};
        lidar_val += lnoise(get_random());
      }
      lidar_scan_data_.ranges.push_back(lidar_val);
    }
    lidar_scan_pub_->publish(lidar_scan_data_);
  }

  /// \brief Main simulation timer loop that publishes markers, tf and timestep
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_;
    timestep_publisher_->publish(message);
    timestep_++;

    if (!draw_only_) {
      //Update the robot config based on wheel command and publish sensor data and tf
      update_config();
      broadcast_turtle();
      send_sensor_data();
    }
  }

  void lidar_timer_callback()
  {
    if (!draw_only_) {
      publish_fake_sensor();
      publish_lidar_scan();
    }
  }

  void check_collision()
  {
    for (size_t i = 0; i < obs_n; i++) {
      double dx = turtlebot_.configuration().x - obstacles_x_.at(i);
      double dy = turtlebot_.configuration().y - obstacles_y_.at(i);
      double d = std::sqrt(std::pow((dx), 2) + std::pow((dy), 2));
      if (d < collision_radius_ + obstacles_r_) {
        turtlebot_.SetConfiguration(turtlelib::Robot_configuration{x0_, y0_, theta0_});
      }
    }
  }

  /// \brief Function for psuedo-random numbers
  std::mt19937 & get_random()
  {
    static std::random_device rd{};
    static std::mt19937 mt{rd()};
    return mt;
  }
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
