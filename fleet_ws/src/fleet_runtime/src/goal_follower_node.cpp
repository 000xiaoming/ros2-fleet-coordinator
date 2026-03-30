#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class GoalFollowerNode : public rclcpp::Node {
public:
  GoalFollowerNode() : Node("goal_follower") {
    declare_parameter<std::string>("robot_id", "robot_1");
    declare_parameter<std::string>("goal_topic", "fleet_goal_pose");
    declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    declare_parameter<std::string>("odom_topic", "odom");
    declare_parameter<std::string>("pose_topic", "amcl_pose");
    declare_parameter<std::string>("navigation_result_topic",
                                   "fleet_navigation_result");
    declare_parameter<double>("goal_tolerance", 0.15);
    declare_parameter<double>("heading_tolerance", 0.2);
    declare_parameter<double>("max_linear_speed", 0.5);
    declare_parameter<double>("max_angular_speed", 1.2);
    declare_parameter<double>("linear_gain", 0.8);
    declare_parameter<double>("angular_gain", 1.5);
    declare_parameter<double>("odom_timeout_sec", 1.5);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        this->get_parameter("cmd_vel_topic").as_string(), 10);
    pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            this->get_parameter("pose_topic").as_string(), 10);
    result_pub_ = this->create_publisher<std_msgs::msg::String>(
        this->get_parameter("navigation_result_topic").as_string(), 10);

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->get_parameter("goal_topic").as_string(), 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          goal_pose_ = *msg;
          has_goal_ = true;
          RCLCPP_INFO(this->get_logger(),
                      "received goal for %s at (%.2f, %.2f)",
                      this->get_parameter("robot_id").as_string().c_str(),
                      goal_pose_.pose.position.x, goal_pose_.pose.position.y);
        });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->get_parameter("odom_topic").as_string(), 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          latest_odom_ = *msg;
          has_odom_ = true;
          last_odom_time_ = std::chrono::steady_clock::now();

          geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
          pose_msg.header = msg->header;
          pose_msg.pose = msg->pose;
          pose_pub_->publish(pose_msg);
        });

    const auto period = std::chrono::duration<double>(0.1);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { control_step(); });
  }

private:
  static double normalize_angle(double angle) {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void publish_result(const std::string & result) {
    std_msgs::msg::String msg;
    msg.data = result;
    result_pub_->publish(msg);
  }

  void stop_robot() {
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
  }

  void control_step() {
    if (!has_odom_) {
      return;
    }

    if (!has_goal_) {
      return;
    }

    const auto timeout =
        std::chrono::duration<double>(this->get_parameter("odom_timeout_sec").as_double());
    if (std::chrono::steady_clock::now() - last_odom_time_ > timeout) {
      stop_robot();
      publish_result("failed");
      has_goal_ = false;
      RCLCPP_WARN(this->get_logger(),
                  "odom timed out while following goal for %s",
                  this->get_parameter("robot_id").as_string().c_str());
      return;
    }

    const auto & pose = latest_odom_.pose.pose;
    const double dx = goal_pose_.pose.position.x - pose.position.x;
    const double dy = goal_pose_.pose.position.y - pose.position.y;
    const double distance = std::hypot(dx, dy);

    if (distance <= this->get_parameter("goal_tolerance").as_double()) {
      stop_robot();
      publish_result("succeeded");
      has_goal_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "goal reached for %s",
                  this->get_parameter("robot_id").as_string().c_str());
      return;
    }

    const double target_yaw = std::atan2(dy, dx);
    const double current_yaw = yaw_from_quaternion(pose.orientation);
    const double yaw_error = normalize_angle(target_yaw - current_yaw);

    geometry_msgs::msg::Twist cmd;
    const double max_linear =
        this->get_parameter("max_linear_speed").as_double();
    const double max_angular =
        this->get_parameter("max_angular_speed").as_double();
    const double linear_gain = this->get_parameter("linear_gain").as_double();
    const double angular_gain = this->get_parameter("angular_gain").as_double();
    const double heading_tolerance =
        this->get_parameter("heading_tolerance").as_double();

    cmd.angular.z = std::clamp(angular_gain * yaw_error, -max_angular, max_angular);
    if (std::fabs(yaw_error) < heading_tolerance) {
      cmd.linear.x = std::clamp(linear_gain * distance, 0.0, max_linear);
    }

    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  nav_msgs::msg::Odometry latest_odom_;
  std::chrono::steady_clock::time_point last_odom_time_{std::chrono::steady_clock::now()};
  bool has_goal_{false};
  bool has_odom_{false};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
