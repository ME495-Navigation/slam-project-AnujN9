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
///
/// SUBSCRIBES:
///     \param /odom (nav_msgs::msg::Odometry): Recieves odom of the robot
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
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&Slam::odom_callback, this, std::placeholders::_1));
    fake_sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "fake_sensor", 10,
      std::bind(&Slam::sensor_callback, this, std::placeholders::_1));

    // Publisher
    estimated_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("estimated_obs", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "green/path", 10);

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //Create the turtlebot
    turtlebot_ = turtlelib::DiffDrive {wheel_radius_, track_width_};

    // Setting the odom message and tf_broadcasted message
    odom_robot_.header.frame_id = odom_id_;
    odom_robot_.child_frame_id = body_id_;
    odom_robot_.transform.translation.z = 0.0;
    map_odom_tf_.header.frame_id = "map";
    map_odom_tf_.child_frame_id = odom_id_;
    map_odom_tf_.transform.translation.z = 0.0;

    // Slam initialization
    prev_state_ = arma::vec{size_, arma::fill::zeros};
    slam_state_ = arma::vec{size_, arma::fill::zeros};
    prev_covariance_ = arma::mat{size_, size_, arma::fill::zeros}; // Covariance
    Q_bar_ = arma::mat{size_, size_, arma::fill::zeros};
    R_ = arma::mat{2 * max_obs_, 2 * max_obs_, arma::fill::zeros};

    Sigma_Q_ = turtlelib::Transform2D(
      turtlelib::Vector2D{
      turtlebot_.configuration().x, turtlebot_.configuration().y},
      turtlebot_.configuration().theta);   // previous odom location

    for (int i = 3; i < size_; i++) {
      prev_covariance_(i, i) = 1e9;
    }

    std::normal_distribution<> noise{0, input_noise_};
    for (int i = 0; i < 2 * max_obs_; i++) {
      R_.at(i, i) = noise(get_random());
    }
  }

private:
  // Variables
  std::string body_id_, odom_id_, wheel_left_, wheel_right_;
  double wheel_radius_, track_width_, input_noise_;
  turtlelib::DiffDrive turtlebot_;
  turtlelib::Wheel pre_pos_{0.0, 0.0};
  turtlelib::Twist2D body_twist_;
  int max_obs_ = 30;
  int iterations = 1;
  double obs_h_ = 0.25, obs_r_ = 0.038;
  turtlelib::Transform2D Tmo_, Tmb_, Tob_;
  turtlelib::Transform2D Sigma_Q_;
  int size_ = 2 * max_obs_ + 3;
  arma::vec prev_state_;
  arma::vec slam_state_;
  arma::mat prev_covariance_;
  arma::mat Q_bar_;
  arma::mat R_;
  std::vector<bool> marker_seen_ = std::vector<bool>(max_obs_, false);

  geometry_msgs::msg::TransformStamped odom_robot_, map_odom_tf_;
  nav_msgs::msg::Path green_path_;

  // Create objects
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub_;
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

  /// \brief Odometry callback which provides current odom reading
  /// \param msg: Odometry message of the robot configuration
  void odom_callback(const nav_msgs::msg::Odometry & msg)
  {
    // Convert the odom msg to twist and current pose
    auto q = msg.pose.pose.orientation;
    auto x = q.x;
    auto y = q.y;
    auto z = q.z;
    auto w = q.w;
    auto a = 2 * (w * z + x * y);
    auto b = 1 - 2 * (y * y + z * z);
    auto z_rot = std::atan2(a, b);
    Tob_ = turtlelib::Transform2D(
      turtlelib::Vector2D{
      msg.pose.pose.position.x, msg.pose.pose.position.y}, z_rot);

    //Publish the tf
    odom_robot_.header.stamp = get_clock()->now();
    odom_robot_.transform.translation.x = Tob_.translation().x;
    odom_robot_.transform.translation.y = Tob_.translation().y;
    odom_robot_.transform.rotation = q;
    tf_broadcaster_->sendTransform(odom_robot_);

    if (iterations % 50 == 0) {
      geometry_msgs::msg::PoseStamped rp;
      green_path_.header.stamp = get_clock()->now();
      green_path_.header.frame_id = odom_id_;
      rp.header.stamp = get_clock()->now();
      rp.header.frame_id = odom_id_;
      rp.pose.position.x = Tob_.translation().x;
      rp.pose.position.y = Tob_.translation().y;
      rp.pose.orientation = q;
      green_path_.poses.push_back(rp);
      path_pub_->publish(green_path_);
      if (iterations == 10000) {
        iterations = 1;
      }
    }
    iterations++;
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

    std::normal_distribution<> noise{0, input_noise_};
    Q_bar_.at(0, 0) = noise(get_random());
    Q_bar_.at(1, 1) = noise(get_random());
    Q_bar_.at(2, 2) = noise(get_random());

    // Covariance prediction
    arma::mat covariance = A * prev_covariance_ * A.t() + Q_bar_;

    for (size_t i = 0; i < msg.markers.size(); i++) {
      const auto action = msg.markers.at(i).action;
      if (action == 2) {
        continue;   // Deleted marker
      }
      const auto dx = msg.markers.at(i).pose.position.x;
      const auto dy = msg.markers.at(i).pose.position.y;
      auto r = sqrt(dx * dx + dy * dy);
      auto phi = turtlelib::normalize_angle(atan2(dy, dx));

      if (!marker_seen_.at(i)) {
        marker_seen_.at(i) = true;
        // Initializing marker measurement
        slam_state_.at(3 + 2 * i) = slam_state_.at(1) + r *
          cos(turtlelib::normalize_angle(phi + slam_state_.at(0)));
        slam_state_.at(3 + 2 * i + 1) = slam_state_.at(2) + r *
          sin(turtlelib::normalize_angle(phi + slam_state_.at(0)));
      }
      // Estimated measure
      auto del_x = slam_state_.at(3 + 2 * i) - slam_state_.at(1);
      auto del_y = slam_state_.at(3 + 2 * i + 1) - slam_state_.at(2);
      auto r_h = del_x * del_x + del_y * del_y;
      auto r_h_rt = sqrt(r_h);
      auto phi_h = turtlelib::normalize_angle(atan2(del_y, del_x) - slam_state_.at(0));

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
      arma::mat K_i = covariance * H_i.t() *
        (H_i * covariance * H_i.t() +
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
      covariance = (I - K_i * H_i) * covariance;
    }
    // Publish odom tf
    Tob_ = turtlelib::Transform2D(
      turtlelib::Vector2D{
      turtlebot_.configuration().x, turtlebot_.configuration().x},
      turtlebot_.configuration().theta);
    Tmb_ = turtlelib::Transform2D(
      turtlelib::Vector2D{
      slam_state_.at(1), slam_state_.at(2)}, slam_state_.at(0));
    Tmo_ = Tmb_ * Tob_.inv();

    map_odom_tf_.header.stamp = get_clock()->now();
    map_odom_tf_.transform.translation.x = Tmo_.translation().x;
    map_odom_tf_.transform.translation.y = Tmo_.translation().y;
    tf2::Quaternion qu;
    qu.setRPY(0, 0, Tmo_.rotation());
    map_odom_tf_.transform.rotation.x = qu.x();
    map_odom_tf_.transform.rotation.y = qu.y();
    map_odom_tf_.transform.rotation.z = qu.z();
    map_odom_tf_.transform.rotation.w = qu.w();
    tf_broadcaster_->sendTransform(map_odom_tf_);

    // Publish estimated markers
    publish_markers();

    //Update variables for next iteration
    prev_state_ = slam_state_;
    prev_covariance_ = covariance;
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
