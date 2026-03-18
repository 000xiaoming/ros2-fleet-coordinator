#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PathPlannerNode : public rclcpp::Node {
public:
  PathPlannerNode() : Node("path_planner") {
    timer_ = this->create_wall_timer(5s, [this]() {
      RCLCPP_INFO(this->get_logger(),
                  "path_planner heartbeat: graph planner scaffold ready");
    });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
