#include <chrono>
#include <memory>
#include <string>

#include "fleet_msgs/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class RobotAgentNode : public rclcpp::Node {
public:
  RobotAgentNode() : Node("robot_agent") {
    declare_parameter<std::string>("robot_id", "robot_1");
    declare_parameter<double>("x", 0.0);
    declare_parameter<double>("y", 0.0);
    state_pub_ =
        this->create_publisher<fleet_msgs::msg::RobotState>("robot_states", 10);

    timer_ = this->create_wall_timer(1s, [this]() {
      fleet_msgs::msg::RobotState state;
      state.robot_id = this->get_parameter("robot_id").as_string();
      state.status = "idle";
      state.battery_percent = 100.0F;
      state.current_task_id = "";
      state.pose.position.x = this->get_parameter("x").as_double();
      state.pose.position.y = this->get_parameter("y").as_double();
      state.pose.orientation.w = 1.0;

      state_pub_->publish(state);

      RCLCPP_INFO(this->get_logger(), "published state for %s at (%.1f, %.1f)",
                  state.robot_id.c_str(), state.pose.position.x,
                  state.pose.position.y);
    });
  }

private:
  rclcpp::Publisher<fleet_msgs::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotAgentNode>());
  rclcpp::shutdown();
  return 0;
}
