#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "fleet_msgs/msg/robot_state.hpp"
#include "fleet_msgs/msg/task.hpp"
#include "fleet_msgs/msg/task_assignment.hpp"
#include "fleet_msgs/msg/task_status.hpp"
#include "fleet_msgs/srv/plan_route.hpp"
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
              !existing->second.current_task_id.empty() &&
              msg->current_task_id.empty() && msg->status == "idle") {
            next_state.status = existing->second.status;
            next_state.current_task_id = existing->second.current_task_id;
          } else if (existing != robot_states_.end() &&
                     msg->status == "completed" &&
                     msg->current_task_id == existing->second.current_task_id) {
            next_state.status = "idle";
            next_state.current_task_id = "";
          }
          robot_states_[msg->robot_id] = next_state;
        });

    assignment_pub_ =
        this->create_publisher<fleet_msgs::msg::TaskAssignment>("task_assignments", 10);
    task_status_pub_ =
        this->create_publisher<fleet_msgs::msg::TaskStatus>("task_statuses", 10);
    plan_route_client_ =
        this->create_client<fleet_msgs::srv::PlanRoute>("plan_route");

    submit_task_srv_ = this->create_service<fleet_msgs::srv::SubmitTask>(
        "submit_task",
        [this](const std::shared_ptr<fleet_msgs::srv::SubmitTask::Request> request,
               std::shared_ptr<fleet_msgs::srv::SubmitTask::Response> response) {
          pending_tasks_.push_back(request->task);

          fleet_msgs::msg::TaskStatus task_status;
          task_status.task_id = request->task.task_id;
          task_status.robot_id = "";
          task_status.status = "queued";
          task_status.message = "task queued by fleet_manager";
          task_status_pub_->publish(task_status);

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
    if (pending_tasks_.empty() || planning_request_in_flight_) {
      return;
    }

    const auto task = pending_tasks_.front();
    const auto robot_id = select_idle_robot();
    if (robot_id.empty()) {
      return;
    }

    const auto & robot = robot_states_.at(robot_id);
    if (robot.current_waypoint.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "robot %s has no current_waypoint; postponing task %s",
                  robot_id.c_str(), task.task_id.c_str());
      return;
    }

    if (!plan_route_client_->wait_for_service(0s)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "plan_route service is not available yet");
      return;
    }

    auto request = std::make_shared<fleet_msgs::srv::PlanRoute::Request>();
    request->start_waypoint = robot.current_waypoint;
    request->pickup_waypoint = task.pickup_waypoint;
    request->dropoff_waypoint = task.dropoff_waypoint;

    planning_request_in_flight_ = true;

    using SharedFuture =
        rclcpp::Client<fleet_msgs::srv::PlanRoute>::SharedFuture;
    plan_route_client_->async_send_request(
        request, [this, task, robot_id](SharedFuture future) {
          planning_request_in_flight_ = false;

          const auto response = future.get();
          if (!response->success || response->route_waypoints.empty()) {
            pending_tasks_.erase(pending_tasks_.begin());

            fleet_msgs::msg::TaskStatus task_status;
            task_status.task_id = task.task_id;
            task_status.robot_id = robot_id;
            task_status.status = "failed";
            task_status.message = response->message;
            task_status_pub_->publish(task_status);

            RCLCPP_WARN(this->get_logger(),
                        "planner rejected task %s for %s; reported failure and dropped task from queue: %s",
                        task.task_id.c_str(), robot_id.c_str(),
                        response->message.c_str());
            return;
          }

          pending_tasks_.erase(pending_tasks_.begin());

          fleet_msgs::msg::TaskAssignment assignment;
          assignment.robot_id = robot_id;
          assignment.task = task;
          assignment.route_waypoints = response->route_waypoints;
          assignment_pub_->publish(assignment);

          fleet_msgs::msg::TaskStatus task_status;
          task_status.task_id = task.task_id;
          task_status.robot_id = robot_id;
          task_status.status = "assigned";
          task_status.message = "task assigned to robot";
          task_status_pub_->publish(task_status);

          auto & assigned_robot = robot_states_.at(robot_id);
          assigned_robot.status = "assigned";
          assigned_robot.current_task_id = task.task_id;
          assigned_robot.current_waypoint = response->route_waypoints.front();

          RCLCPP_INFO(this->get_logger(),
                      "assigned task %s to %s with %zu route waypoints",
                      task.task_id.c_str(), robot_id.c_str(),
                      response->route_waypoints.size());
        });
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
  bool planning_request_in_flight_{false};
  rclcpp::Subscription<fleet_msgs::msg::RobotState>::SharedPtr robot_state_sub_;
  rclcpp::Publisher<fleet_msgs::msg::TaskAssignment>::SharedPtr assignment_pub_;
  rclcpp::Publisher<fleet_msgs::msg::TaskStatus>::SharedPtr task_status_pub_;
  rclcpp::Client<fleet_msgs::srv::PlanRoute>::SharedPtr plan_route_client_;
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
