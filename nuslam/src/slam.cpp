/// \file
/// \brief The slam node uses EKF slam to determine the turtlebot aand obstacle locations
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
///     \param /estimated (visualization_msgs::msg::MarkerArray):
///     \param /green/path (nav_msgs::msg::Path): Publishes path of the green robot
///     \param /odom (nav_msgs::msg::Odometry): Publishes odometry state
///
/// SUBSCRIBES:
///     \param /joint_states (sensor_msgs::msg:JointState): Recieves joint states to update odom
///     \param /fake_sensor (visualization_msgs::msg::MarkerArray): Recieves sensor data of
///              the robot
///
/// SERVERS:
///     None
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
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "armadillo"

using namespace std::chrono_literals;

class Slam : public rclcpp::Node
{
  /// \brief Main class that deals with Slam of the turtlebot.
  ///
  /// \param wheel_radius (double): The radius of the wheels [m]
  /// \param track_width (double): The distance between the wheels [m]
  /// \param body_id (std::string): The name of the body frame of the robot
  /// \param odom_id (std::string): The name of the odometry frame
  /// \param wheel_left (std::string): The name of the left wheel joint
  /// \param wheel_right (std::string): The name of the right wheel joint

public:
  Slam()
  : Node("Slam")
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
    param_desc.description = "Input noise of the system";
    declare_parameter("input_noise", rclcpp::PARAMETER_DOUBLE, param_desc);

    check_parameters();

    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    body_id_ = get_parameter("body_id").get_parameter_value().get<std::string>();
    odom_id_ = get_parameter("odom_id").get_parameter_value().get<std::string>();
    wheel_left_ = get_parameter("wheel_left").get_parameter_value().get<std::string>();
    wheel_right_ = get_parameter("wheel_right").get_parameter_value().get<std::string>();
    input_noise_ = get_parameter("input_noise").get_parameter_value().get<double>();

    // Subscriber
    fake_sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "fake_sensor", 10,
      std::bind(&Slam::sensor_callback, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&Slam::joint_state_callback, this, std::placeholders::_1));

    // Publisher
    estimated_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("estimated_obs", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "green/path", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //Create the turtlebot
    turtlebot_ = turtlelib::DiffDrive {wheel_radius_, track_width_};

    // Setting the odom message and tf_broadcasted message
    odom_robot_.header.frame_id = odom_id_;
    odom_robot_.child_frame_id = body_id_;
    odom_robot_.transform.translation.z = 0.0;
    nav_odom_msg_.header.frame_id = odom_id_;
    nav_odom_msg_.child_frame_id = body_id_;
    nav_odom_msg_.pose.pose.position.z = 0.0;
    map_odom_tf_.header.frame_id = "map";
    map_odom_tf_.child_frame_id = odom_id_;
    map_odom_tf_.transform.translation.z = 0.0;

    // Slam initialization
    prev_state_ = arma::vec{size_, arma::fill::zeros};
    slam_state_ = arma::vec{size_, arma::fill::zeros};
    covariance_ = arma::mat{size_, size_, arma::fill::zeros}; // Covariance

    Q_bar_ = arma::mat{size_, size_, arma::fill::zeros};
    Q_bar_.at(0, 0) = 1e-3; // THANK YOU CARTER
    Q_bar_.at(1, 1) = 1e-3;
    Q_bar_.at(2, 2) = 1e-3;
    R_ = arma::mat{static_cast<unsigned long int>(2 * max_obs_),
      static_cast<unsigned long int>(2 * max_obs_), arma::fill::zeros};

    Sigma_Q_ = turtlelib::Transform2D(
      turtlelib::Vector2D{
      turtlebot_.configuration().x, turtlebot_.configuration().y},
      turtlebot_.configuration().theta);   // previous odom location

    for (int i = 3; i < static_cast<int>(size_); i++) {
      covariance_(i, i) = 1e6;
    }

    for (int i = 0; i < 2 * max_obs_; i++) {
      R_.at(i, i) = 1e-3;  // THANK YOU CARTER
    }
  }

private:
  // Variables
  std::string body_id_, odom_id_, wheel_left_, wheel_right_;
  double wheel_radius_, track_width_, input_noise_;
  turtlelib::DiffDrive turtlebot_;
  turtlelib::Robot_configuration Q_;
  turtlelib::Wheel pre_pos_{0.0, 0.0};
  turtlelib::Twist2D body_twist_;
  int max_obs_ = 30;
  int iterations = 1;
  double obs_h_ = 0.25, obs_r_ = 0.038;
  turtlelib::Transform2D Tmo_, Tmb_, Tob_;
  turtlelib::Transform2D Sigma_Q_;
  unsigned long size_ = 2 * max_obs_ + 3;
  arma::vec prev_state_;
  arma::vec slam_state_;
  arma::mat covariance_;
  arma::mat Q_bar_;
  arma::mat R_;
  std::vector<bool> marker_seen_ = std::vector<bool>(max_obs_, false);

  nav_msgs::msg::Odometry nav_odom_msg_;
  tf2::Quaternion quaternion_;
  geometry_msgs::msg::TransformStamped odom_robot_, map_odom_tf_;
  nav_msgs::msg::Path green_path_;

  // Create objects
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr estimated_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
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
  /// \param msg: Joint state message that updates the robot configuration
  void joint_state_callback(const sensor_msgs::msg::JointState & msg)
  {
    // Getting the twist and updating the robot config
    body_twist_ = turtlebot_.ForwardKinematics(
      {msg.position.at(0) - pre_pos_.left, msg.position.at(1) - pre_pos_.right});
    Q_ = turtlebot_.configuration();

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
    odom_robot_.header.stamp = get_clock()->now();
    odom_robot_.transform.translation.x = turtlebot_.configuration().x;
    odom_robot_.transform.translation.y = turtlebot_.configuration().y;
    odom_robot_.transform.rotation.x = quaternion_.x();
    odom_robot_.transform.rotation.y = quaternion_.y();
    odom_robot_.transform.rotation.z = quaternion_.z();
    odom_robot_.transform.rotation.w = quaternion_.w();
    tf_broadcaster_->sendTransform(odom_robot_);

    // Update the previous wheel position
    pre_pos_.left = msg.position.at(0);
    pre_pos_.right = msg.position.at(1);
  }

  /// \brief Fake sensor callback where cextended Kalman filter SLAM is calculated
  /// \param msg - Marker Array which represent the fake sensor measurements
  void sensor_callback(const visualization_msgs::msg::MarkerArray & msg)
  {
    // Kalman Prediction
    Tob_ = turtlelib::Transform2D(
      turtlelib::Vector2D{
      turtlebot_.configuration().x, turtlebot_.configuration().y},
      turtlebot_.configuration().theta);
    Tmb_ = Tmo_ * Tob_;

    // Calculation of state prediction
    slam_state_.at(0) = turtlelib::normalize_angle(Tmb_.rotation());
    slam_state_.at(1) = Tmb_.translation().x;
    slam_state_.at(2) = Tmb_.translation().y;

    arma::mat A = arma::eye<arma::mat>(size_, size_);
    A.at(1, 0) += -(slam_state_.at(2) - prev_state_.at(2));
    A.at(2, 0) += (slam_state_.at(1) - prev_state_.at(1));

    // Covariance prediction
    covariance_ = A * covariance_ * A.t() + Q_bar_;

    for (size_t i = 0; i < msg.markers.size(); i++) {
      const auto action = msg.markers.at(i).action;
      if (action == 2) {
        continue;   // Deleted marker
      }
      const auto dx = msg.markers.at(i).pose.position.x;
      const auto dy = msg.markers.at(i).pose.position.y;
      auto r = std::sqrt(dx * dx + dy * dy);
      auto phi = turtlelib::normalize_angle(std::atan2(dy, dx));

      if (!marker_seen_.at(i)) {
        marker_seen_.at(i) = true;
        // Initializing marker measurement
        slam_state_.at(3 + 2 * i) = slam_state_.at(1) + r *
          std::cos(turtlelib::normalize_angle(phi + slam_state_.at(0)));
        slam_state_.at(3 + 2 * i + 1) = slam_state_.at(2) + r *
          std::sin(turtlelib::normalize_angle(phi + slam_state_.at(0)));
      }
      // Estimated measure
      auto del_x = slam_state_.at(3 + 2 * i) - slam_state_.at(1);
      auto del_y = slam_state_.at(3 + 2 * i + 1) - slam_state_.at(2);
      auto r_h = del_x * del_x + del_y * del_y;
      auto r_h_rt = std::sqrt(r_h);
      auto phi_h = turtlelib::normalize_angle(std::atan2(del_y, del_x) - slam_state_.at(0));

      // H matrix formation
      arma::mat H_i {2, size_, arma::fill::zeros};
      H_i.at(1, 0) = -1.0;

      H_i.at(0, 1) = -del_x / r_h_rt;
      H_i.at(1, 1) = del_y / r_h;
      H_i.at(0, 2) = -del_y / r_h_rt;
      H_i.at(1, 2) = -del_x / r_h;

      H_i.at(0, 3 + 2 * i) = del_x / r_h_rt;
      H_i.at(1, 3 + 2 * i) = -del_y / r_h;
      H_i.at(0, 3 + 2 * i + 1) = del_y / r_h_rt;
      H_i.at(1, 3 + 2 * i + 1) = del_x / r_h;

      // Kalmin filter gain
      arma::mat K_i = covariance_ * H_i.t() *
        (H_i * covariance_ * H_i.t() +
        R_.submat(2 * i, 2 * i, 2 * i + 1, 2 * i + 1)).i();

      // State Update
      arma::vec z(2, arma::fill::zeros);
      z.at(0) = r;
      z.at(1) = phi;

      arma::vec z_h(2, arma::fill::zeros);
      z_h.at(0) = r_h_rt;
      z_h.at(1) = phi_h;

      arma::vec del_z = z - z_h;
      del_z.at(1) = turtlelib::normalize_angle(del_z.at(1));
      slam_state_ = slam_state_ + K_i * del_z;
      slam_state_.at(0) = turtlelib::normalize_angle(slam_state_.at(0));

      // Covariance Update
      arma::mat I = arma::eye<arma::mat>(size_, size_);
      covariance_ = (I - K_i * H_i) * covariance_;
    }
    // Publish odom tf
    Tob_ = turtlelib::Transform2D(
      turtlelib::Vector2D{
      turtlebot_.configuration().x, turtlebot_.configuration().y},
      turtlebot_.configuration().theta);
    Tmb_ = turtlelib::Transform2D(
      turtlelib::Vector2D{
      slam_state_.at(1), slam_state_.at(2)}, slam_state_.at(0));
    Tmo_ = Tmb_ * Tob_.inv();

    map_odom_tf_.header.stamp = get_clock()->now();
    map_odom_tf_.transform.translation.x = Tmo_.translation().x;
    map_odom_tf_.transform.translation.y = Tmo_.translation().y;
    tf2::Quaternion qu;
    qu.setRPY(0, 0, turtlelib::normalize_angle(Tmo_.rotation()));
    map_odom_tf_.transform.rotation.x = qu.x();
    map_odom_tf_.transform.rotation.y = qu.y();
    map_odom_tf_.transform.rotation.z = qu.z();
    map_odom_tf_.transform.rotation.w = qu.w();
    tf_broadcaster_->sendTransform(map_odom_tf_);

    // Publish estimated markers and path
    publish_markers();
    publish_path();

    //Update variables for next iteration
    prev_state_ = slam_state_;
  }

  /// \brief Publishes the markers estimated by slam
  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray ma;
    for (int i = 0; i < max_obs_; i++) {
      if (marker_seen_.at(i)) {
        visualization_msgs::msg::Marker obj;
        obj.header.stamp = get_clock()->now();
        obj.header.frame_id = "map";
        obj.id = i;
        obj.type = 3;
        obj.color.r = 0.0;
        obj.color.g = 1.0;
        obj.color.b = 0.0;
        obj.color.a = 1.0;
        obj.scale.x = 2 * obs_r_;
        obj.scale.y = 2 * obs_r_;
        obj.scale.z = 0.25;
        obj.pose.position.x = slam_state_.at(3 + 2 * i);
        obj.pose.position.y = slam_state_.at(3 + 2 * i + 1);
        obj.pose.position.z = 0.125;
        ma.markers.push_back(obj);
      }
    }
    estimated_pub_->publish(ma);
  }

  /// \brief Publishes the green path
  void publish_path()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = get_clock()->now();
    pose.pose.position.x = slam_state_.at(1);
    pose.pose.position.y = slam_state_.at(2);
    pose.pose.position.z = 0.0;
    green_path_.header.stamp = get_clock()->now();
    green_path_.header.frame_id = "nusim/world";
    green_path_.poses.push_back(pose);
    path_pub_->publish(green_path_);
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
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}
