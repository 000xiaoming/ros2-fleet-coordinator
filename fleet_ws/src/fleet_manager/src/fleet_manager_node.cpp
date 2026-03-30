#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
    reservation_timeout_ =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(
                this->declare_parameter("reservation_timeout_sec", 5.0)));
    waypoint_positions_ = load_waypoint_positions();

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
            release_task_reservation(msg->current_task_id);
            next_state.status = "idle";
            next_state.current_task_id = "";
          } else if (existing != robot_states_.end() &&
                     msg->status == "failed" &&
                     msg->current_task_id == existing->second.current_task_id) {
            release_task_reservation(msg->current_task_id);
            next_state.status = "idle";
            next_state.current_task_id = "";
          }
          robot_states_[msg->robot_id] = next_state;
          robot_last_update_[msg->robot_id] = std::chrono::steady_clock::now();
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
          publish_task_status(request->task.task_id, "", "queued",
                              "task queued by fleet_manager");

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
      cleanup_stale_robot_assignments();
      RCLCPP_INFO(this->get_logger(),
                  "tracking %zu robots, %zu pending tasks, %zu reserved tasks",
                  robot_states_.size(), pending_tasks_.size(),
                  task_reservations_.size());
    });
  }

private:
  using TaskIterator = std::vector<fleet_msgs::msg::Task>::iterator;

  struct PlanningAttempt {
    fleet_msgs::msg::Task task;
    std::vector<std::string> candidate_robot_ids;
    std::size_t next_candidate_index{0};
    bool saw_reservation_block{false};
    std::string last_failure_message;
  };

  std::unordered_map<std::string, std::pair<double, double>>
  load_waypoint_positions() {
    std::unordered_map<std::string, std::pair<double, double>> positions;

    this->declare_parameter<std::vector<std::string>>("waypoint_positions",
                                                      std::vector<std::string>{});
    const auto configured_positions =
        this->get_parameter("waypoint_positions").as_string_array();
    for (const auto & entry : configured_positions) {
      const auto first = entry.find(':');
      const auto second =
          entry.find(':', first == std::string::npos ? first : first + 1);
      if (first == std::string::npos || second == std::string::npos ||
          first == 0 || second <= first + 1 || second + 1 >= entry.size()) {
        RCLCPP_WARN(this->get_logger(),
                    "ignoring malformed waypoint position '%s'",
                    entry.c_str());
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

  void publish_task_status(const std::string & task_id,
                           const std::string & robot_id,
                           const std::string & status,
                           const std::string & message) {
    const auto signature = robot_id + "|" + status + "|" + message;
    const auto existing = last_task_status_.find(task_id);
    if (existing != last_task_status_.end() && existing->second == signature) {
      return;
    }

    last_task_status_[task_id] = signature;

    fleet_msgs::msg::TaskStatus task_status;
    task_status.task_id = task_id;
    task_status.robot_id = robot_id;
    task_status.status = status;
    task_status.message = message;
    task_status_pub_->publish(task_status);
  }

  void release_task_reservation(const std::string & task_id) {
    task_reservations_.erase(task_id);
  }

  void cleanup_stale_robot_assignments() {
    const auto now = std::chrono::steady_clock::now();
    std::vector<std::string> stale_robot_ids;

    for (const auto & [robot_id, state] : robot_states_) {
      const auto update_it = robot_last_update_.find(robot_id);
      if (update_it == robot_last_update_.end()) {
        continue;
      }

      if (now - update_it->second <= reservation_timeout_) {
        continue;
      }

      stale_robot_ids.push_back(robot_id);

      if (!state.current_task_id.empty()) {
        release_task_reservation(state.current_task_id);
        publish_task_status(state.current_task_id, robot_id, "failed",
                            "task failed because robot state updates timed out");

        RCLCPP_WARN(this->get_logger(),
                    "released reservation for task %s after robot %s timed out",
                    state.current_task_id.c_str(), robot_id.c_str());
      }
    }

    for (const auto & robot_id : stale_robot_ids) {
      robot_states_.erase(robot_id);
      robot_last_update_.erase(robot_id);
    }
  }

  std::vector<std::string> reserved_waypoints() const {
    std::vector<std::string> reserved;
    std::unordered_set<std::string> seen;

    for (const auto & [task_id, waypoints] : task_reservations_) {
      (void)task_id;
      for (const auto & waypoint : waypoints) {
        if (seen.insert(waypoint).second) {
          reserved.push_back(waypoint);
        }
      }
    }

    return reserved;
  }

  TaskIterator find_pending_task(const std::string & task_id) {
    return std::find_if(
        pending_tasks_.begin(), pending_tasks_.end(),
        [&task_id](const fleet_msgs::msg::Task & candidate) {
          return candidate.task_id == task_id;
        });
  }

  TaskIterator select_pending_task() {
    if (pending_tasks_.empty()) {
      return pending_tasks_.end();
    }

    return std::max_element(
        pending_tasks_.begin(), pending_tasks_.end(),
        [](const fleet_msgs::msg::Task & lhs, const fleet_msgs::msg::Task & rhs) {
          return lhs.priority < rhs.priority;
        });
  }

  double squared_distance_to_pickup(
      const fleet_msgs::msg::RobotState & robot,
      const fleet_msgs::msg::Task & task) const {
    const auto pickup_it = waypoint_positions_.find(task.pickup_waypoint);
    if (pickup_it == waypoint_positions_.end()) {
      return std::numeric_limits<double>::infinity();
    }

    const auto dx = robot.pose.position.x - pickup_it->second.first;
    const auto dy = robot.pose.position.y - pickup_it->second.second;
    return dx * dx + dy * dy;
  }

  std::vector<std::string>
  ranked_idle_robots_for_task(const fleet_msgs::msg::Task & task) const {
    struct CandidateRobot {
      std::string robot_id;
      double distance_score;
      float battery_percent;
    };

    std::vector<CandidateRobot> candidates;
    for (const auto & [robot_id, state] : robot_states_) {
      if (state.status != "idle" || state.current_waypoint.empty()) {
        continue;
      }

      candidates.push_back(CandidateRobot{robot_id,
                                          squared_distance_to_pickup(state, task),
                                          state.battery_percent});
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const CandidateRobot & lhs, const CandidateRobot & rhs) {
                if (lhs.distance_score != rhs.distance_score) {
                  return lhs.distance_score < rhs.distance_score;
                }
                if (lhs.battery_percent != rhs.battery_percent) {
                  return lhs.battery_percent > rhs.battery_percent;
                }
                return lhs.robot_id < rhs.robot_id;
              });

    std::vector<std::string> ranked_robot_ids;
    ranked_robot_ids.reserve(candidates.size());
    for (const auto & candidate : candidates) {
      ranked_robot_ids.push_back(candidate.robot_id);
    }
    return ranked_robot_ids;
  }

  void finish_planning_attempt() {
    current_planning_attempt_.reset();
    planning_request_in_flight_ = false;
  }

  void finalize_exhausted_planning_attempt() {
    if (!current_planning_attempt_) {
      planning_request_in_flight_ = false;
      return;
    }

    const auto task = current_planning_attempt_->task;
    auto pending_task = find_pending_task(task.task_id);
    if (pending_task == pending_tasks_.end()) {
      finish_planning_attempt();
      return;
    }

    if (current_planning_attempt_->saw_reservation_block) {
      if (std::next(pending_task) != pending_tasks_.end()) {
        std::rotate(pending_task, std::next(pending_task), pending_tasks_.end());
      }
      publish_task_status(task.task_id, "", "waiting",
                          "task is waiting for reserved waypoints to clear");

      RCLCPP_INFO(this->get_logger(),
                  "planner postponed task %s after all candidate robots were blocked by reservations",
                  task.task_id.c_str());
      finish_planning_attempt();
      return;
    }

    if (!current_planning_attempt_->last_failure_message.empty()) {
      const auto message = current_planning_attempt_->last_failure_message;
      pending_tasks_.erase(pending_task);
      publish_task_status(task.task_id, "", "failed", message);

      RCLCPP_WARN(this->get_logger(),
                  "planner rejected task %s for all candidate robots; dropped task from queue: %s",
                  task.task_id.c_str(), message.c_str());
      finish_planning_attempt();
      return;
    }

    RCLCPP_WARN(this->get_logger(),
                "all candidate robots for task %s became unavailable while planning; leaving task queued",
                task.task_id.c_str());
    finish_planning_attempt();
  }

  bool request_next_candidate_plan() {
    if (!current_planning_attempt_) {
      planning_request_in_flight_ = false;
      return false;
    }

    while (current_planning_attempt_->next_candidate_index <
           current_planning_attempt_->candidate_robot_ids.size()) {
      const auto robot_id = current_planning_attempt_->candidate_robot_ids
          [current_planning_attempt_->next_candidate_index++];
      const auto robot_it = robot_states_.find(robot_id);
      if (robot_it == robot_states_.end() || robot_it->second.status != "idle" ||
          robot_it->second.current_waypoint.empty() ||
          !robot_it->second.current_task_id.empty()) {
        continue;
      }

      auto request = std::make_shared<fleet_msgs::srv::PlanRoute::Request>();
      request->start_waypoint = robot_it->second.current_waypoint;
      request->pickup_waypoint = current_planning_attempt_->task.pickup_waypoint;
      request->dropoff_waypoint = current_planning_attempt_->task.dropoff_waypoint;
      request->reserved_waypoints = reserved_waypoints();

      planning_request_in_flight_ = true;

      using SharedFuture =
          rclcpp::Client<fleet_msgs::srv::PlanRoute>::SharedFuture;
      plan_route_client_->async_send_request(
          request, [this, robot_id](SharedFuture future) {
            const auto response = future.get();
            const auto task_id =
                current_planning_attempt_ ? current_planning_attempt_->task.task_id : "";
            if (!current_planning_attempt_) {
              planning_request_in_flight_ = false;
              return;
            }

            auto pending_task =
                find_pending_task(current_planning_attempt_->task.task_id);
            if (pending_task == pending_tasks_.end()) {
              RCLCPP_WARN(this->get_logger(),
                          "planner result arrived for task %s after it left the queue",
                          task_id.c_str());
              finish_planning_attempt();
              return;
            }

            if (!response->success || response->route_waypoints.empty()) {
              if (response->message == "route blocked by reserved waypoints") {
                current_planning_attempt_->saw_reservation_block = true;
                planning_request_in_flight_ = false;
                if (!request_next_candidate_plan()) {
                  finalize_exhausted_planning_attempt();
                }
                return;
              }

              current_planning_attempt_->last_failure_message = response->message;
              planning_request_in_flight_ = false;
              if (!request_next_candidate_plan()) {
                finalize_exhausted_planning_attempt();
              }
              return;
            }

            const auto robot_it = robot_states_.find(robot_id);
            if (robot_it == robot_states_.end() ||
                robot_it->second.status != "idle" ||
                !robot_it->second.current_task_id.empty()) {
              RCLCPP_WARN(this->get_logger(),
                          "planner succeeded for task %s but robot %s is no longer idle; trying another candidate",
                          task_id.c_str(), robot_id.c_str());
              planning_request_in_flight_ = false;
              if (!request_next_candidate_plan()) {
                finalize_exhausted_planning_attempt();
              }
              return;
            }

            const auto task = current_planning_attempt_->task;
            pending_tasks_.erase(pending_task);

            fleet_msgs::msg::TaskAssignment assignment;
            assignment.robot_id = robot_id;
            assignment.task = task;
            assignment.route_waypoints = response->route_waypoints;
            assignment_pub_->publish(assignment);

            task_reservations_[task.task_id] = response->route_waypoints;
            publish_task_status(task.task_id, robot_id, "assigned",
                                "task assigned to robot");

            auto & assigned_robot = robot_it->second;
            assigned_robot.status = "assigned";
            assigned_robot.current_task_id = task.task_id;
            assigned_robot.current_waypoint = response->route_waypoints.front();
            robot_last_update_[robot_id] = std::chrono::steady_clock::now();

            RCLCPP_INFO(this->get_logger(),
                        "assigned task %s to %s with %zu route waypoints",
                        task.task_id.c_str(), robot_id.c_str(),
                        response->route_waypoints.size());
            finish_planning_attempt();
          });
      return true;
    }

    planning_request_in_flight_ = false;
    return false;
  }

  void assign_pending_tasks() {
    cleanup_stale_robot_assignments();

    if (pending_tasks_.empty() || planning_request_in_flight_) {
      return;
    }

    const auto pending_task = select_pending_task();
    if (pending_task == pending_tasks_.end()) {
      return;
    }

    const auto task = *pending_task;
    const auto candidate_robot_ids = ranked_idle_robots_for_task(task);
    if (candidate_robot_ids.empty()) {
      return;
    }

    if (!plan_route_client_->wait_for_service(0s)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "plan_route service is not available yet");
      return;
    }

    current_planning_attempt_ =
        PlanningAttempt{task, candidate_robot_ids, 0, false, ""};
    if (!request_next_candidate_plan()) {
      finalize_exhausted_planning_attempt();
    }
  }

  std::unordered_map<std::string, fleet_msgs::msg::RobotState> robot_states_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> robot_last_update_;
  std::vector<fleet_msgs::msg::Task> pending_tasks_;
  std::unordered_map<std::string, std::vector<std::string>> task_reservations_;
  std::unordered_map<std::string, std::string> last_task_status_;
  std::unordered_map<std::string, std::pair<double, double>> waypoint_positions_;
  std::optional<PlanningAttempt> current_planning_attempt_;
  std::chrono::steady_clock::duration reservation_timeout_{std::chrono::seconds(5)};
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
