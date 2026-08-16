// Copyright 2026 LiberNav contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <memory>

#include "libernav_rog_map/rog_map_ros.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.enable_rosout(false);
  auto node = std::make_shared<libernav_rog_map::RogMapROS>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
