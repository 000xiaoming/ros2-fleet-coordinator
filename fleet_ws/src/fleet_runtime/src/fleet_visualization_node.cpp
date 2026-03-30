#include <array>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "fleet_msgs/msg/robot_state.hpp"
#include "fleet_msgs/msg/task_assignment.hpp"
#include "fleet_msgs/msg/task_status.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class FleetVisualizationNode : public rclcpp::Node {
public:
  FleetVisualizationNode() : Node("fleet_visualization") {
    declare_parameter<std::string>("map_frame", "world");
    declare_parameter<std::vector<std::string>>("waypoint_positions",
                                                std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("graph_edges",
                                                std::vector<std::string>{});

    waypoint_positions_ = load_waypoint_positions();
    graph_edges_ = load_graph_edges();

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "fleet_markers", 10);

    robot_state_sub_ = this->create_subscription<fleet_msgs::msg::RobotState>(
        "robot_states", 10,
        [this](const fleet_msgs::msg::RobotState::SharedPtr msg) {
          robot_states_[msg->robot_id] = *msg;
        });

    assignment_sub_ = this->create_subscription<fleet_msgs::msg::TaskAssignment>(
        "task_assignments", 10,
        [this](const fleet_msgs::msg::TaskAssignment::SharedPtr msg) {
          assignments_[msg->robot_id] = *msg;
        });

    task_status_sub_ = this->create_subscription<fleet_msgs::msg::TaskStatus>(
        "task_statuses", 10,
        [this](const fleet_msgs::msg::TaskStatus::SharedPtr msg) {
          task_statuses_[msg->task_id] = *msg;
        });

    publish_timer_ = this->create_wall_timer(1s, [this]() { publish_markers(); });
  }

private:
  std::unordered_map<std::string, std::pair<double, double>>
  load_waypoint_positions() {
    std::unordered_map<std::string, std::pair<double, double>> positions;
    const auto configured_positions =
        this->get_parameter("waypoint_positions").as_string_array();
    for (const auto & entry : configured_positions) {
      const auto first = entry.find(':');
      const auto second =
          entry.find(':', first == std::string::npos ? first : first + 1);
      if (first == std::string::npos || second == std::string::npos ||
          first == 0 || second <= first + 1 || second + 1 >= entry.size()) {
        continue;
      }

      try {
        positions[entry.substr(0, first)] = {
            std::stod(entry.substr(first + 1, second - first - 1)),
            std::stod(entry.substr(second + 1))};
      } catch (const std::exception &) {
      }
    }
    return positions;
  }

  std::vector<std::pair<std::string, std::string>> load_graph_edges() {
    std::vector<std::pair<std::string, std::string>> edges;
    const auto configured_edges =
        this->get_parameter("graph_edges").as_string_array();
    for (const auto & entry : configured_edges) {
      const auto separator = entry.find(':');
      if (separator == std::string::npos || separator == 0 ||
          separator + 1 >= entry.size()) {
        continue;
      }
      edges.emplace_back(entry.substr(0, separator), entry.substr(separator + 1));
    }
    return edges;
  }

  static geometry_msgs::msg::Point make_point(double x, double y, double z) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  }

  static std::array<float, 4> robot_color(const std::string & status) {
    if (status == "executing") {
      return {1.0F, 0.6F, 0.0F, 1.0F};
    }
    if (status == "completed") {
      return {0.0F, 0.6F, 1.0F, 1.0F};
    }
    if (status == "failed") {
      return {1.0F, 0.0F, 0.0F, 1.0F};
    }
    if (status == "assigned") {
      return {0.8F, 0.8F, 0.0F, 1.0F};
    }
    return {0.1F, 0.8F, 0.2F, 1.0F};
  }

  visualization_msgs::msg::Marker base_marker(
      int id, const std::string & ns, int type) const {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = this->get_parameter("map_frame").as_string();
    marker.header.stamp = this->now();
    marker.id = id;
    marker.ns = ns;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  void publish_markers() {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear_all;
    clear_all.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_all);

    int id = 0;

    auto graph_marker = base_marker(id++, "graph_edges",
                                    visualization_msgs::msg::Marker::LINE_LIST);
    graph_marker.scale.x = 0.03;
    graph_marker.color.r = 0.5F;
    graph_marker.color.g = 0.5F;
    graph_marker.color.b = 0.5F;
    graph_marker.color.a = 1.0F;
    for (const auto & edge : graph_edges_) {
      const auto from = waypoint_positions_.find(edge.first);
      const auto to = waypoint_positions_.find(edge.second);
      if (from == waypoint_positions_.end() || to == waypoint_positions_.end()) {
        continue;
      }
      graph_marker.points.push_back(make_point(from->second.first, from->second.second, 0.05));
      graph_marker.points.push_back(make_point(to->second.first, to->second.second, 0.05));
    }
    markers.markers.push_back(graph_marker);

    for (const auto & [waypoint, position] : waypoint_positions_) {
      auto waypoint_marker =
          base_marker(id++, "waypoints", visualization_msgs::msg::Marker::SPHERE);
      waypoint_marker.pose.position.x = position.first;
      waypoint_marker.pose.position.y = position.second;
      waypoint_marker.pose.position.z = 0.1;
      waypoint_marker.scale.x = 0.15;
      waypoint_marker.scale.y = 0.15;
      waypoint_marker.scale.z = 0.15;
      waypoint_marker.color.r = 0.0F;
      waypoint_marker.color.g = 0.7F;
      waypoint_marker.color.b = 1.0F;
      waypoint_marker.color.a = 1.0F;
      markers.markers.push_back(waypoint_marker);

      auto text_marker =
          base_marker(id++, "waypoint_labels", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      text_marker.pose.position.x = position.first;
      text_marker.pose.position.y = position.second;
      text_marker.pose.position.z = 0.35;
      text_marker.scale.z = 0.18;
      text_marker.color.r = 1.0F;
      text_marker.color.g = 1.0F;
      text_marker.color.b = 1.0F;
      text_marker.color.a = 1.0F;
      text_marker.text = waypoint;
      markers.markers.push_back(text_marker);
    }

    for (const auto & [robot_id, state] : robot_states_) {
      const auto color = robot_color(state.status);

      auto robot_marker =
          base_marker(id++, "robots", visualization_msgs::msg::Marker::CYLINDER);
      robot_marker.pose = state.pose;
      robot_marker.pose.position.z = 0.12;
      robot_marker.scale.x = 0.28;
      robot_marker.scale.y = 0.28;
      robot_marker.scale.z = 0.22;
      robot_marker.color.r = color[0];
      robot_marker.color.g = color[1];
      robot_marker.color.b = color[2];
      robot_marker.color.a = color[3];
      markers.markers.push_back(robot_marker);

      auto text_marker =
          base_marker(id++, "robot_labels", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      text_marker.pose.position.x = state.pose.position.x;
      text_marker.pose.position.y = state.pose.position.y;
      text_marker.pose.position.z = 0.55;
      text_marker.scale.z = 0.18;
      text_marker.color.r = 1.0F;
      text_marker.color.g = 1.0F;
      text_marker.color.b = 1.0F;
      text_marker.color.a = 1.0F;
      text_marker.text = robot_id + " " + state.status;
      if (!state.current_task_id.empty()) {
        const auto task_status = task_statuses_.find(state.current_task_id);
        if (task_status != task_statuses_.end()) {
          text_marker.text += "\\n" + task_status->second.status;
        } else {
          text_marker.text += "\\n" + state.current_task_id;
        }
      }
      markers.markers.push_back(text_marker);

      const auto assignment = assignments_.find(robot_id);
      if (assignment == assignments_.end()) {
        continue;
      }

      const auto task_status = task_statuses_.find(assignment->second.task.task_id);
      if (task_status != task_statuses_.end() &&
          (task_status->second.status == "completed" ||
           task_status->second.status == "failed")) {
        continue;
      }

      auto route_marker =
          base_marker(id++, "routes", visualization_msgs::msg::Marker::LINE_STRIP);
      route_marker.scale.x = 0.05;
      route_marker.color.r = 1.0F;
      route_marker.color.g = 0.2F;
      route_marker.color.b = 0.2F;
      route_marker.color.a = 1.0F;
      route_marker.points.push_back(
          make_point(state.pose.position.x, state.pose.position.y, 0.08));
      for (const auto & waypoint : assignment->second.route_waypoints) {
        const auto waypoint_position = waypoint_positions_.find(waypoint);
        if (waypoint_position == waypoint_positions_.end()) {
          continue;
        }
        route_marker.points.push_back(make_point(waypoint_position->second.first,
                                                 waypoint_position->second.second,
                                                 0.08));
      }
      if (route_marker.points.size() > 1) {
        markers.markers.push_back(route_marker);
      }
    }

    marker_pub_->publish(markers);
  }

  std::unordered_map<std::string, std::pair<double, double>> waypoint_positions_;
  std::vector<std::pair<std::string, std::string>> graph_edges_;
  std::unordered_map<std::string, fleet_msgs::msg::RobotState> robot_states_;
  std::unordered_map<std::string, fleet_msgs::msg::TaskAssignment> assignments_;
  std::unordered_map<std::string, fleet_msgs::msg::TaskStatus> task_statuses_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<fleet_msgs::msg::RobotState>::SharedPtr robot_state_sub_;
  rclcpp::Subscription<fleet_msgs::msg::TaskAssignment>::SharedPtr assignment_sub_;
  rclcpp::Subscription<fleet_msgs::msg::TaskStatus>::SharedPtr task_status_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FleetVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
