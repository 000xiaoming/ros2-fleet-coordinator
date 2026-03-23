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
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class RobotAgentNode : public rclcpp::Node {
public:
  RobotAgentNode() : Node("robot_agent") {
    declare_parameter<std::string>("robot_id", "robot_1");
    declare_parameter<double>("x", 0.0);
    declare_parameter<double>("y", 0.0);
    declare_parameter<std::string>("start_waypoint", "dock_a");
    declare_parameter<std::vector<std::string>>("waypoint_positions",
                                                std::vector<std::string>{});
    waypoint_positions_ = load_waypoint_positions();
    state_pub_ =
        this->create_publisher<fleet_msgs::msg::RobotState>("robot_states", 10);
    task_status_pub_ =
        this->create_publisher<fleet_msgs::msg::TaskStatus>("task_statuses", 10);
    assignment_sub_ = this->create_subscription<fleet_msgs::msg::TaskAssignment>(
        "task_assignments", 10,
        [this](const fleet_msgs::msg::TaskAssignment::SharedPtr msg) {
          const auto robot_id = this->get_parameter("robot_id").as_string();
          if (msg->robot_id != robot_id) {
            return;
          }
          if (active_task_) {
            RCLCPP_WARN(this->get_logger(),
                        "ignoring assignment %s because %s is already busy",
                        msg->task.task_id.c_str(), robot_id.c_str());
            return;
          }
          if (msg->route_waypoints.empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "ignoring empty route for task %s",
                        msg->task.task_id.c_str());
            return;
          }

          active_task_ = true;
          completion_announced_ = false;
          active_task_id_ = msg->task.task_id;
          route_waypoints_ = msg->route_waypoints;
          route_index_ = 0;
          current_waypoint_ = route_waypoints_.front();

          publish_task_status("executing",
                              "task accepted by robot and execution started");

          RCLCPP_INFO(this->get_logger(),
                      "accepted task %s with %zu route waypoints",
                      active_task_id_.c_str(), route_waypoints_.size());
        });

    current_waypoint_ = this->get_parameter("start_waypoint").as_string();

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
      const auto second = entry.find(':', first == std::string::npos ? first : first + 1);
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

  void publish_state() {
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
      active_task_ = false;
      active_task_id_.clear();
      route_waypoints_.clear();
      route_index_ = 0;
      state.status = "idle";
      state.current_task_id = "";
    }

    const auto position = lookup_waypoint_position(state.current_waypoint);
    state.pose.position.x = position.first;
    state.pose.position.y = position.second;
    state.pose.orientation.w = 1.0;

    state_pub_->publish(state);

    if (clear_completed_task) {
      active_task_ = false;
      completion_announced_ = false;
      active_task_id_.clear();
      route_waypoints_.clear();
      route_index_ = 0;
    }

    RCLCPP_INFO(this->get_logger(),
                "published %s state for %s at %s (%.1f, %.1f)",
                state.status.c_str(), state.robot_id.c_str(),
                state.current_waypoint.c_str(), state.pose.position.x,
                state.pose.position.y);
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
  rclcpp::Subscription<fleet_msgs::msg::TaskAssignment>::SharedPtr assignment_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool active_task_{false};
  bool completion_announced_{false};
  std::string active_task_id_;
  std::string current_waypoint_;
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
