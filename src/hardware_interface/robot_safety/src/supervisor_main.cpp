#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_safety/supervisor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_safety::SafetySupervisor>());
  rclcpp::shutdown();
  return 0;
}
