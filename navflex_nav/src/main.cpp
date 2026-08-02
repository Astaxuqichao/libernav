#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "navflex_nav/costmap_nav/navflex_costmap_nav.hpp"
#include "navflex_nav/rogmap_nav/navflex_rogmap_nav.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto selector = std::make_shared<rclcpp::Node>("navflex_nav");
  selector->declare_parameter("navigation_type", "costmap");
  const std::string navigation_type = selector->get_parameter("navigation_type").as_string();
  selector.reset();

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=navflex_nav"});
  std::vector<std::shared_ptr<nav2_util::LifecycleNode>> navigation_nodes;
  if (navigation_type == "costmap") {
    navigation_nodes.push_back(std::make_shared<navflex_nav::CostmapNavNode>(options));
  } else if (navigation_type == "rogmap") {
    navigation_nodes.push_back(std::make_shared<navflex_nav::RogMapNavNode>(options));
  } else if (navigation_type == "both") {
    navigation_nodes.push_back(std::make_shared<navflex_nav::CostmapNavNode>(options));
    rclcpp::NodeOptions rogmap_options;
    rogmap_options.arguments(
      {"--ros-args", "-r", "__node:=navflex_nav", "-r", "__ns:=/rogmap"});
    navigation_nodes.push_back(std::make_shared<navflex_nav::RogMapNavNode>(rogmap_options));
  } else {
    RCLCPP_FATAL(
      rclcpp::get_logger("navflex_nav"),
      "Invalid navigation_type '%s'; expected 'costmap', 'rogmap', or 'both'",
      navigation_type.c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("navflex_nav"), "Starting navflex_nav in %s mode with %zu backend(s)",
    navigation_type.c_str(), navigation_nodes.size());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);
  for (const auto & navigation_node : navigation_nodes) {
    executor.add_node(navigation_node->get_node_base_interface());
  }
  executor.spin();
  navigation_nodes.clear();
  rclcpp::shutdown();
  return 0;
}
