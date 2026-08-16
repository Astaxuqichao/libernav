// Copyright 2026 Navflex contributors
// SPDX-License-Identifier: BSD-3-Clause

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "navflex_scan_planner/single_scan_planner.hpp"

namespace
{

navflex_rog_map::RogMapConfig mapConfig()
{
  navflex_rog_map::RogMapConfig config;
  config.global_resolution = 0.1;
  config.local_resolution = 0.05;
  config.global_bounds = {-10.0, -10.0, -2.0, 10.0, 10.0, 5.0};
  config.local_size_x = 10.0;
  config.local_size_y = 10.0;
  config.local_size_z = 5.0;
  config.inflation_resolution = 0.05;
  config.inflation_step = 1;
  config.unknown_inflation = false;
  config.enable_esdf = true;
  config.esdf_update_interval = 1;
  config.point_filter_num = 1;
  config.global_point_filter_num = 1;
  config.batch_update_size = 1;
  return config;
}

navflex_rog_map::Footprint3D footprint()
{
  navflex_rog_map::Footprint3D result;
  result.type = navflex_rog_map::FootprintType::SPHERE;
  result.radius = 0.12;
  return result;
}

nav_msgs::msg::Path straightPath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i <= 8; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = i * 0.25;
    pose.pose.position.z = 1.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

geometry_msgs::msg::Pose startPose()
{
  geometry_msgs::msg::Pose pose;
  pose.position.z = 1.0;
  pose.orientation.w = 1.0;
  return pose;
}

}  // namespace

TEST(SingleScanPlanner, ProducesTimedTrajectoryFromReference)
{
  auto map = std::make_shared<navflex_rog_map::RogMap>(mapConfig());
  geometry_msgs::msg::Point origin;
  origin.z = 1.0;
  std::vector<geometry_msgs::msg::Point> ray_end(1);
  ray_end.front().x = 4.0;
  ray_end.front().z = 1.0;
  map->update(origin, ray_end);
  navflex_scan_planner::SingleScanPlannerConfig config;
  config.optimization_iterations = 5;
  navflex_scan_planner::SingleScanPlanner planner(config, map, footprint());
  navflex_scan_planner::TimedScanTrajectory trajectory;
  std::string message;

  ASSERT_TRUE(planner.plan(straightPath(), startPose(), trajectory, message)) << message;
  ASSERT_GE(trajectory.points.size(), 2u);
  EXPECT_EQ(trajectory.points.size(), trajectory.times.size());
  EXPECT_GT(trajectory.duration, 0.0);
  for (size_t i = 1; i < trajectory.times.size(); ++i) {
    EXPECT_GT(trajectory.times[i], trajectory.times[i - 1]);
  }
  EXPECT_NEAR(trajectory.points.front().x, 0.0, 1e-6);
  EXPECT_NEAR(trajectory.points.back().x, 2.0, 1e-6);
}

TEST(SingleScanPlanner, RepairsBlockedReferenceWithConstrainedAStar)
{
  auto map = std::make_shared<navflex_rog_map::RogMap>(mapConfig());
  geometry_msgs::msg::Point origin;
  origin.z = 1.0;
  std::vector<geometry_msgs::msg::Point> obstacle(1);
  obstacle.front().x = 1.0;
  obstacle.front().z = 1.0;
  map->update(origin, obstacle);

  navflex_scan_planner::SingleScanPlannerConfig config;
  config.optimization_iterations = 5;
  config.astar_search_margin = 1.0;
  config.max_reference_deviation = 1.0;
  navflex_scan_planner::SingleScanPlanner planner(config, map, footprint());
  navflex_scan_planner::TimedScanTrajectory trajectory;
  std::string message;

  ASSERT_TRUE(planner.plan(straightPath(), startPose(), trajectory, message)) << message;
  bool deviated = false;
  for (const auto & point : trajectory.points) {
    deviated = deviated || std::abs(point.y) > 0.05;
  }
  EXPECT_TRUE(deviated);
}

TEST(SingleScanPlanner, RepairsCollisionOnFinalReferenceEdge)
{
  auto map = std::make_shared<navflex_rog_map::RogMap>(mapConfig());
  geometry_msgs::msg::Point origin;
  origin.z = 1.0;
  std::vector<geometry_msgs::msg::Point> obstacle(1);
  obstacle.front().x = 1.85;
  obstacle.front().z = 1.0;
  map->update(origin, obstacle);

  navflex_scan_planner::SingleScanPlannerConfig config;
  config.optimization_iterations = 5;
  config.astar_search_margin = 1.0;
  config.max_reference_deviation = 1.0;
  auto small_footprint = footprint();
  small_footprint.radius = 0.03;
  navflex_scan_planner::SingleScanPlanner planner(config, map, small_footprint);
  navflex_scan_planner::TimedScanTrajectory trajectory;
  std::string message;

  ASSERT_TRUE(planner.plan(straightPath(), startPose(), trajectory, message)) << message;
  EXPECT_NEAR(trajectory.points.back().x, 2.0, 1e-6);
}
