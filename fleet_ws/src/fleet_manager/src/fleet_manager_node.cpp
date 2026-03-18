#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "fleet_msgs/msg/robot_state.hpp"
#include "fleet_msgs/msg/task.hpp"
#include "fleet_msgs/srv/submit_task.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class FleetManagerNode : public rclcpp::Node {
public:
  FleetManagerNode() : Node("fleet_manager") {
    robot_state_sub_ = this->create_subscription<fleet_msgs::msg::RobotState>(
        "robot_states", 10,
        [this](const fleet_msgs::msg::RobotState::SharedPtr msg) {
          auto next_state = *msg;
          const auto existing = robot_states_.find(msg->robot_id);
          if (existing != robot_states_.end() &&
              !existing->second.current_task_id.empty()) {
            next_state.status = existing->second.status;
            next_state.current_task_id = existing->second.current_task_id;
          }
          robot_states_[msg->robot_id] = next_state;
        });

    submit_task_srv_ = this->create_service<fleet_msgs::srv::SubmitTask>(
        "submit_task",
        [this](const std::shared_ptr<fleet_msgs::srv::SubmitTask::Request> request,
               std::shared_ptr<fleet_msgs::srv::SubmitTask::Response> response) {
          pending_tasks_.push_back(request->task);
          response->accepted = true;
          response->message = "task queued";
          RCLCPP_INFO(this->get_logger(),
                      "queued task %s from %s to %s (priority=%u)",
                      request->task.task_id.c_str(),
                      request->task.pickup_waypoint.c_str(),
                      request->task.dropoff_waypoint.c_str(),
                      request->task.priority);
        });

    assign_timer_ = this->create_wall_timer(1s, [this]() { assign_pending_tasks(); });
    summary_timer_ = this->create_wall_timer(5s, [this]() {
      RCLCPP_INFO(this->get_logger(), "tracking %zu robots, %zu pending tasks",
                  robot_states_.size(), pending_tasks_.size());
    });
  }

private:
  void assign_pending_tasks() {
    if (pending_tasks_.empty()) {
      return;
    }

    auto task = pending_tasks_.front();
    const auto robot_id = select_idle_robot();
    if (robot_id.empty()) {
      return;
    }

    pending_tasks_.erase(pending_tasks_.begin());

    auto & robot = robot_states_.at(robot_id);
    robot.status = "assigned";
    robot.current_task_id = task.task_id;

    RCLCPP_INFO(this->get_logger(),
                "assigned task %s to %s: %s -> %s",
                task.task_id.c_str(), robot_id.c_str(),
                task.pickup_waypoint.c_str(), task.dropoff_waypoint.c_str());
  }

  std::string select_idle_robot() const {
    for (const auto & [robot_id, state] : robot_states_) {
      if (state.status == "idle") {
        return robot_id;
      }
    }
    return "";
  }

  std::unordered_map<std::string, fleet_msgs::msg::RobotState> robot_states_;
  std::vector<fleet_msgs::msg::Task> pending_tasks_;
  rclcpp::Subscription<fleet_msgs::msg::RobotState>::SharedPtr robot_state_sub_;
  rclcpp::Service<fleet_msgs::srv::SubmitTask>::SharedPtr submit_task_srv_;
  rclcpp::TimerBase::SharedPtr assign_timer_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FleetManagerNode>());
  rclcpp::shutdown();
  return 0;
}
