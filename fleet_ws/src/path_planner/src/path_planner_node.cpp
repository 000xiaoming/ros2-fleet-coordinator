#include <algorithm>
#include <chrono>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "fleet_msgs/srv/plan_route.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PathPlannerNode : public rclcpp::Node {
public:
  PathPlannerNode() : Node("path_planner") {
    declare_parameter<std::vector<std::string>>(
        "graph_waypoints",
        {"dock_a", "mid_1", "pickup_zone", "mid_2", "dropoff_zone", "dock_c"});
    declare_parameter<std::vector<std::string>>(
        "graph_edges",
        {"dock_a:mid_1", "mid_1:pickup_zone", "mid_1:mid_2",
         "mid_2:dropoff_zone", "mid_2:dock_c"});
    load_graph_from_parameters();

    plan_route_srv_ = this->create_service<fleet_msgs::srv::PlanRoute>(
        "plan_route",
        [this](const std::shared_ptr<fleet_msgs::srv::PlanRoute::Request> request,
               std::shared_ptr<fleet_msgs::srv::PlanRoute::Response> response) {
          const auto to_pickup =
              shortest_path(request->start_waypoint, request->pickup_waypoint);
          const auto to_dropoff =
              shortest_path(request->pickup_waypoint, request->dropoff_waypoint);

          if (to_pickup.empty() || to_dropoff.empty()) {
            response->success = false;
            response->message = "unable to plan route on configured graph";
            return;
          }

          response->success = true;
          response->message = "route planned";
          response->route_waypoints = to_pickup;
          response->route_waypoints.insert(response->route_waypoints.end(),
                                           std::next(to_dropoff.begin()),
                                           to_dropoff.end());

          RCLCPP_INFO(this->get_logger(),
                      "planned route %s -> %s -> %s (%zu waypoints)",
                      request->start_waypoint.c_str(),
                      request->pickup_waypoint.c_str(),
                      request->dropoff_waypoint.c_str(),
                      response->route_waypoints.size());
        });

    timer_ = this->create_wall_timer(5s, [this]() {
      RCLCPP_INFO(this->get_logger(),
                  "path_planner ready with %zu configured waypoints",
                  graph_.size());
    });
  }

private:
  void load_graph_from_parameters() {
    graph_.clear();

    const auto waypoints =
        this->get_parameter("graph_waypoints").as_string_array();
    for (const auto & waypoint : waypoints) {
      graph_[waypoint] = {};
    }

    const auto edges = this->get_parameter("graph_edges").as_string_array();
    for (const auto & edge : edges) {
      const auto separator = edge.find(':');
      if (separator == std::string::npos || separator == 0 ||
          separator + 1 >= edge.size()) {
        RCLCPP_WARN(this->get_logger(),
                    "ignoring malformed graph edge '%s'", edge.c_str());
        continue;
      }

      const auto from = edge.substr(0, separator);
      const auto to = edge.substr(separator + 1);
      graph_[from].push_back(to);
      graph_[to].push_back(from);
    }

    if (graph_.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "planner graph is empty after parameter loading");
    }
  }

  std::vector<std::string> shortest_path(const std::string & start,
                                         const std::string & goal) const {
    if (!graph_.count(start) || !graph_.count(goal)) {
      return {};
    }

    std::queue<std::string> frontier;
    std::unordered_map<std::string, std::string> parent;
    frontier.push(start);
    parent[start] = "";

    while (!frontier.empty()) {
      const auto current = frontier.front();
      frontier.pop();

      if (current == goal) {
        break;
      }

      for (const auto & neighbor : graph_.at(current)) {
        if (parent.count(neighbor)) {
          continue;
        }
        parent[neighbor] = current;
        frontier.push(neighbor);
      }
    }

    if (!parent.count(goal)) {
      return {};
    }

    std::vector<std::string> path;
    for (std::string node = goal; !node.empty(); node = parent.at(node)) {
      path.push_back(node);
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  std::unordered_map<std::string, std::vector<std::string>> graph_;
  rclcpp::Service<fleet_msgs::srv::PlanRoute>::SharedPtr plan_route_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
