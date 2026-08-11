// Copyright 2026 Navflex contributors
// SPDX-License-Identifier: BSD-3-Clause

#ifndef SCAN_PLANNER__SINGLE_SCAN_PLANNER_HPP_
#define SCAN_PLANNER__SINGLE_SCAN_PLANNER_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navflex_rog_map/footprint.hpp"
#include "navflex_rog_map/rog_map.hpp"

namespace scan_planner
{

struct ScanPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct SingleScanPlannerConfig
{
  double sample_distance{0.1};
  double control_point_distance{0.2};
  double astar_resolution{0.15};
  double astar_search_margin{2.0};
  int astar_max_expansions{30000};
  int optimization_iterations{60};
  double optimization_step_size{0.08};
  double smooth_weight{1.0};
  double collision_weight{1.0};
  double reference_weight{1.0};
  double feasibility_weight{0.1};
  double collision_distance{0.2};
  double max_reference_deviation{1.5};
  double max_velocity{0.75};
  double max_acceleration{0.5};
};

struct TimedScanTrajectory
{
  std::vector<ScanPoint> points;
  std::vector<double> times;
  double duration{0.0};
};

class SingleScanPlanner
{
public:
  SingleScanPlanner(
    SingleScanPlannerConfig config, navflex_rog_map::RogMap::ConstPtr map,
    navflex_rog_map::Footprint3D footprint);

  bool plan(
    const nav_msgs::msg::Path & reference, const geometry_msgs::msg::Pose & start,
    TimedScanTrajectory & trajectory, std::string & message) const;

private:
  using Points = std::vector<ScanPoint>;

  Points prepareReference(
    const nav_msgs::msg::Path & reference, const geometry_msgs::msg::Pose & start) const;
  bool repairCollisions(Points & points, std::string & message) const;
  bool repairSegment(
    const ScanPoint & start, const ScanPoint & goal, Points & replacement) const;
  void optimize(Points & points) const;
  Points sampleCubicBspline(const Points & control_points) const;
  void parameterize(const Points & points, TimedScanTrajectory & trajectory) const;
  bool poseFree(const ScanPoint & point, double yaw) const;
  bool segmentFree(const ScanPoint & start, const ScanPoint & goal) const;
  double footprintClearance() const;

  SingleScanPlannerConfig config_;
  navflex_rog_map::RogMap::ConstPtr map_;
  navflex_rog_map::Footprint3D footprint_;
};

}  // namespace scan_planner

#endif  // SCAN_PLANNER__SINGLE_SCAN_PLANNER_HPP_
