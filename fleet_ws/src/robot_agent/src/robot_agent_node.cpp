#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "fleet_msgs/msg/robot_state.hpp"
#include "fleet_msgs/msg/task_assignment.hpp"
#include "fleet_msgs/msg/task_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class RobotAgentNode : public rclcpp::Node {
public:
  RobotAgentNode() : Node("robot_agent") {
    declare_parameter<std::string>("robot_id", "robot_1");
    declare_parameter<double>("x", 0.0);
    declare_parameter<double>("y", 0.0);
    declare_parameter<std::string>("start_waypoint", "dock_a");
    declare_parameter<std::string>("execution_mode", "simulated");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("pose_topic", "amcl_pose");
    declare_parameter<std::string>("goal_topic", "fleet_goal_pose");
    declare_parameter<std::string>("navigation_result_topic",
                                   "fleet_navigation_result");
    declare_parameter<std::vector<std::string>>("waypoint_positions",
                                                std::vector<std::string>{});

    waypoint_positions_ = load_waypoint_positions();
    current_waypoint_ = this->get_parameter("start_waypoint").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    execution_mode_ = this->get_parameter("execution_mode").as_string();
    if (execution_mode_ == "nav2") {
      execution_mode_ = "external_navigation";
    }

    state_pub_ =
        this->create_publisher<fleet_msgs::msg::RobotState>("robot_states", 10);
    task_status_pub_ =
        this->create_publisher<fleet_msgs::msg::TaskStatus>("task_statuses", 10);

    assignment_sub_ = this->create_subscription<fleet_msgs::msg::TaskAssignment>(
        "task_assignments", 10,
        [this](const fleet_msgs::msg::TaskAssignment::SharedPtr msg) {
          handle_assignment(*msg);
        });

    if (execution_mode_ == "external_navigation") {
      pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          this->get_parameter("pose_topic").as_string(), 10,
          [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            last_pose_ = msg->pose.pose;
            has_pose_ = true;
          });
      goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
          this->get_parameter("goal_topic").as_string(), 10);
      navigation_result_sub_ = this->create_subscription<std_msgs::msg::String>(
          this->get_parameter("navigation_result_topic").as_string(), 10,
          [this](const std_msgs::msg::String::SharedPtr msg) {
            handle_navigation_result(msg->data);
          });
    } else if (execution_mode_ != "simulated") {
      RCLCPP_WARN(this->get_logger(),
                  "unknown execution_mode '%s'; falling back to simulated mode",
                  execution_mode_.c_str());
      execution_mode_ = "simulated";
    }

    timer_ = this->create_wall_timer(1s, [this]() {
      publish_state();
    });
  }

private:
  std::unordered_map<std::string, std::pair<double, double>>
  load_waypoint_positions() {
    std::unordered_map<std::string, std::pair<double, double>> positions = {
        {"dock_a", {0.0, 0.0}},
        {"mid_1", {1.0, 0.0}},
        {"pickup_zone", {2.0, 0.0}},
        {"mid_2", {3.0, 0.5}},
        {"dropoff_zone", {4.0, 1.0}},
        {"dock_c", {3.0, 1.5}},
    };

    const auto configured_positions =
        this->get_parameter("waypoint_positions").as_string_array();
    for (const auto & entry : configured_positions) {
      const auto first = entry.find(':');
      const auto second =
          entry.find(':', first == std::string::npos ? first : first + 1);
      if (first == std::string::npos || second == std::string::npos ||
          first == 0 || second <= first + 1 || second + 1 >= entry.size()) {
        RCLCPP_WARN(this->get_logger(),
                    "ignoring malformed waypoint position '%s'", entry.c_str());
        continue;
      }

      const auto waypoint = entry.substr(0, first);

      try {
        const auto x = std::stod(entry.substr(first + 1, second - first - 1));
        const auto y = std::stod(entry.substr(second + 1));
        positions[waypoint] = {x, y};
      } catch (const std::exception &) {
        RCLCPP_WARN(this->get_logger(),
                    "ignoring malformed numeric waypoint position '%s'",
                    entry.c_str());
      }
    }

    return positions;
  }

  void handle_assignment(const fleet_msgs::msg::TaskAssignment & assignment) {
    const auto robot_id = this->get_parameter("robot_id").as_string();
    if (assignment.robot_id != robot_id) {
      return;
    }
    if (active_task_) {
      RCLCPP_WARN(this->get_logger(),
                  "ignoring assignment %s because %s is already busy",
                  assignment.task.task_id.c_str(), robot_id.c_str());
      return;
    }
    if (assignment.route_waypoints.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "ignoring empty route for task %s",
                  assignment.task.task_id.c_str());
      return;
    }

    active_task_ = true;
    completion_announced_ = false;
    navigation_goal_in_flight_ = false;
    terminal_state_.clear();
    active_task_id_ = assignment.task.task_id;
    route_waypoints_ = assignment.route_waypoints;
    route_index_ = 0;
    current_waypoint_ = route_waypoints_.front();

    publish_task_status("accepted", "task accepted by robot agent");

    RCLCPP_INFO(this->get_logger(),
                "accepted task %s with %zu route waypoints in %s mode",
                active_task_id_.c_str(), route_waypoints_.size(),
                execution_mode_.c_str());

    if (execution_mode_ == "external_navigation") {
      publish_navigation_goal();
      return;
    }

    publish_task_status("executing",
                        "task accepted by robot and execution started");
  }

  void publish_navigation_goal() {
    if (!active_task_ || route_index_ >= route_waypoints_.size()) {
      return;
    }

    if (!goal_pub_) {
      fail_active_task("navigation goal publisher is not configured");
      return;
    }

    const auto target_waypoint = route_waypoints_[route_index_];
    const auto target_position = lookup_waypoint_position(target_waypoint);

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = map_frame_;
    goal.header.stamp = this->now();
    goal.pose.position.x = target_position.first;
    goal.pose.position.y = target_position.second;
    goal.pose.orientation.w = 1.0;

    current_waypoint_ = target_waypoint;
    navigation_goal_in_flight_ = true;
    goal_pub_->publish(goal);

    publish_task_status("executing",
                        "published navigation goal for waypoint " + target_waypoint);
  }

  void handle_navigation_result(const std::string & result) {
    if (!active_task_ || execution_mode_ != "external_navigation") {
      return;
    }

    navigation_goal_in_flight_ = false;

    if (result == "succeeded") {
      current_waypoint_ = route_waypoints_[route_index_];
      ++route_index_;
      if (route_index_ < route_waypoints_.size()) {
        publish_navigation_goal();
        return;
      }
      publish_task_status("completed",
                          "external navigation reported task completion");
      terminal_state_ = "completed";
      return;
    }

    if (result == "aborted" || result == "canceled" || result == "failed") {
      fail_active_task("external navigation reported " + result + " for waypoint " +
                       route_waypoints_[route_index_]);
      return;
    }

    RCLCPP_WARN(this->get_logger(),
                "ignoring unknown navigation result '%s' for task %s",
                result.c_str(), active_task_id_.c_str());
  }

  void publish_task_status(const std::string & status,
                           const std::string & message) {
    if (active_task_id_.empty()) {
      return;
    }

    fleet_msgs::msg::TaskStatus task_status;
    task_status.task_id = active_task_id_;
    task_status.robot_id = this->get_parameter("robot_id").as_string();
    task_status.status = status;
    task_status.message = message;
    task_status_pub_->publish(task_status);
  }

  void fail_active_task(const std::string & message) {
    publish_task_status("failed", message);
    RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
    navigation_goal_in_flight_ = false;
    terminal_state_ = "failed";
  }

  void clear_active_task() {
    active_task_ = false;
    completion_announced_ = false;
    navigation_goal_in_flight_ = false;
    terminal_state_.clear();
    active_task_id_.clear();
    route_waypoints_.clear();
    route_index_ = 0;
  }

  void publish_state() {
    if (execution_mode_ == "external_navigation") {
      publish_navigation_backed_state();
      return;
    }

    publish_simulated_state();
  }

  void publish_navigation_backed_state() {
    fleet_msgs::msg::RobotState state;
    state.robot_id = this->get_parameter("robot_id").as_string();
    state.battery_percent = 100.0F;
    state.current_waypoint = current_waypoint_;
    state.pose = current_pose();

    if (!active_task_) {
      state.status = "idle";
      state.current_task_id = "";
    } else if (!terminal_state_.empty()) {
      state.status = terminal_state_;
      state.current_task_id = active_task_id_;
    } else {
      state.status = navigation_goal_in_flight_ ? "executing" : "assigned";
      state.current_task_id = active_task_id_;
    }

    state_pub_->publish(state);

    RCLCPP_INFO(this->get_logger(),
                "published %s state for %s at (%.2f, %.2f) targeting %s",
                state.status.c_str(), state.robot_id.c_str(),
                state.pose.position.x, state.pose.position.y,
                state.current_waypoint.c_str());

    if (!terminal_state_.empty()) {
      clear_active_task();
    }
  }

  void publish_simulated_state() {
    fleet_msgs::msg::RobotState state;
    state.robot_id = this->get_parameter("robot_id").as_string();
    state.battery_percent = 100.0F;
    state.current_waypoint = current_waypoint_;
    bool clear_completed_task = false;

    if (!active_task_) {
      state.status = "idle";
      state.current_task_id = "";
    } else if (route_index_ + 1 < route_waypoints_.size()) {
      ++route_index_;
      current_waypoint_ = route_waypoints_[route_index_];
      state.status = "executing";
      state.current_task_id = active_task_id_;
      state.current_waypoint = current_waypoint_;
    } else if (!completion_announced_) {
      completion_announced_ = true;
      state.status = "completed";
      state.current_task_id = active_task_id_;
      publish_task_status("completed", "task execution completed");
      clear_completed_task = true;
    } else {
      state.status = "idle";
      state.current_task_id = "";
      clear_completed_task = true;
    }

    const auto position = lookup_waypoint_position(state.current_waypoint);
    state.pose.position.x = position.first;
    state.pose.position.y = position.second;
    state.pose.orientation.w = 1.0;

    state_pub_->publish(state);

    if (clear_completed_task) {
      clear_active_task();
    }

    RCLCPP_INFO(this->get_logger(),
                "published %s state for %s at %s (%.1f, %.1f)",
                state.status.c_str(), state.robot_id.c_str(),
                state.current_waypoint.c_str(), state.pose.position.x,
                state.pose.position.y);
  }

  geometry_msgs::msg::Pose current_pose() const {
    if (has_pose_) {
      return last_pose_;
    }

    geometry_msgs::msg::Pose pose;
    const auto position = lookup_waypoint_position(current_waypoint_);
    pose.position.x = position.first;
    pose.position.y = position.second;
    pose.orientation.w = 1.0;
    return pose;
  }

  std::pair<double, double>
  lookup_waypoint_position(const std::string & waypoint) const {
    const auto it = waypoint_positions_.find(waypoint);
    if (it != waypoint_positions_.end()) {
      return it->second;
    }

    return {this->get_parameter("x").as_double(),
            this->get_parameter("y").as_double()};
  }

  rclcpp::Publisher<fleet_msgs::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::Publisher<fleet_msgs::msg::TaskStatus>::SharedPtr task_status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<fleet_msgs::msg::TaskAssignment>::SharedPtr assignment_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_result_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool active_task_{false};
  bool completion_announced_{false};
  bool navigation_goal_in_flight_{false};
  bool has_pose_{false};
  std::string active_task_id_;
  std::string current_waypoint_;
  std::string execution_mode_;
  std::string map_frame_;
  std::string terminal_state_;
  geometry_msgs::msg::Pose last_pose_;
  std::unordered_map<std::string, std::pair<double, double>> waypoint_positions_;
  std::vector<std::string> route_waypoints_;
  std::size_t route_index_{0};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotAgentNode>());
  rclcpp::shutdown();
  return 0;
}
