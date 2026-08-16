// Copyright 2026 Navflex contributors
// SPDX-License-Identifier: BSD-3-Clause

#include "navflex_scan_planner/single_scan_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>

#include "tf2/LinearMath/Quaternion.h"

namespace navflex_scan_planner
{
namespace
{

double distance(const ScanPoint & a, const ScanPoint & b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

ScanPoint add(const ScanPoint & a, const ScanPoint & b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

ScanPoint subtract(const ScanPoint & a, const ScanPoint & b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

ScanPoint scale(const ScanPoint & point, double value)
{
  return {point.x * value, point.y * value, point.z * value};
}

double norm(const ScanPoint & point)
{
  return std::hypot(std::hypot(point.x, point.y), point.z);
}

geometry_msgs::msg::Point toMessage(const ScanPoint & point)
{
  geometry_msgs::msg::Point result;
  result.x = point.x;
  result.y = point.y;
  result.z = point.z;
  return result;
}

geometry_msgs::msg::Pose makePose(const ScanPoint & point, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position = toMessage(point);
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

int64_t latticeKey(int x, int y)
{
  return (static_cast<int64_t>(x) << 32) ^ static_cast<uint32_t>(y);
}

}  // namespace

SingleScanPlanner::SingleScanPlanner(
  SingleScanPlannerConfig config, navflex_rog_map::RogMap::ConstPtr map,
  navflex_rog_map::Footprint3D footprint)
: config_(std::move(config)), map_(std::move(map)), footprint_(std::move(footprint))
{
}

bool SingleScanPlanner::plan(
  const nav_msgs::msg::Path & reference, const geometry_msgs::msg::Pose & start,
  TimedScanTrajectory & trajectory, std::string & message) const
{
  trajectory = TimedScanTrajectory();
  Points points = prepareReference(reference, start);
  if (points.size() < 2) {
    message = "Reference path has fewer than two usable points";
    return false;
  }
  if (!repairCollisions(points, message)) {
    return false;
  }

  optimize(points);
  Points sampled = sampleCubicBspline(points);
  bool spline_is_free = sampled.size() >= 2;
  for (size_t i = 1; spline_is_free && i < sampled.size(); ++i) {
    spline_is_free = segmentFree(sampled[i - 1], sampled[i]);
  }
  if (!spline_is_free) {
    // Optimization is soft-constrained. Retain the repaired collision-free path
    // if smoothing cuts a corner too aggressively.
    sampled = points;
  }
  for (size_t i = 1; i < sampled.size(); ++i) {
    if (!segmentFree(sampled[i - 1], sampled[i])) {
      message = "Optimized SCAN trajectory is not collision free";
      return false;
    }
  }

  parameterize(sampled, trajectory);
  if (trajectory.points.size() < 2 || trajectory.duration <= 0.0) {
    message = "Failed to time-parameterize SCAN trajectory";
    return false;
  }
  message = "Single-shot SCAN trajectory planned from RogAStar reference";
  return true;
}

SingleScanPlanner::Points SingleScanPlanner::prepareReference(
  const nav_msgs::msg::Path & reference, const geometry_msgs::msg::Pose & start) const
{
  Points raw;
  raw.reserve(reference.poses.size());
  for (const auto & pose : reference.poses) {
    const auto & p = pose.pose.position;
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      raw.push_back({p.x, p.y, p.z});
    }
  }
  if (raw.empty()) {
    return {};
  }

  const ScanPoint current{start.position.x, start.position.y, start.position.z};
  size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < raw.size(); ++i) {
    const double candidate = distance(current, raw[i]);
    if (candidate < nearest_distance) {
      nearest_distance = candidate;
      nearest = i;
    }
  }

  Points result{current};
  double carried = 0.0;
  ScanPoint previous_raw = current;
  for (size_t i = nearest; i < raw.size(); ++i) {
    const double step = distance(previous_raw, raw[i]);
    carried += step;
    if (carried + 1e-9 >= config_.control_point_distance || i + 1 == raw.size()) {
      if (distance(result.back(), raw[i]) > 1e-6) {
        result.push_back(raw[i]);
      }
      carried = 0.0;
    }
    previous_raw = raw[i];
  }
  return result;
}

bool SingleScanPlanner::repairCollisions(Points & points, std::string & message) const
{
  Points repaired{points.front()};
  size_t index = 1;
  while (index < points.size()) {
    if (segmentFree(repaired.back(), points[index])) {
      repaired.push_back(points[index++]);
      continue;
    }

    size_t reconnect = index;
    while (reconnect < points.size() && !poseFree(
        points[reconnect], std::atan2(
          points[reconnect].y - repaired.back().y,
          points[reconnect].x - repaired.back().x)))
    {
      ++reconnect;
    }
    if (reconnect >= points.size()) {
      message = "RogAStar reference terminates inside the current ROG map obstacle model";
      return false;
    }

    Points detour;
    if (!repairSegment(repaired.back(), points[reconnect], detour)) {
      message = "Reference-constrained local A* could not repair a colliding path segment";
      return false;
    }
    repaired.insert(repaired.end(), std::next(detour.begin()), detour.end());
    index = reconnect + 1;
  }
  points = std::move(repaired);
  return true;
}

bool SingleScanPlanner::repairSegment(
  const ScanPoint & start, const ScanPoint & goal, Points & replacement) const
{
  struct Node
  {
    int x{0};
    int y{0};
    double g{0.0};
    double f{0.0};
    int64_t parent{0};
    bool has_parent{false};
    bool closed{false};
  };
  struct QueueEntry
  {
    double score;
    int64_t key;
    bool operator<(const QueueEntry & other) const {return score > other.score;}
  };

  const double resolution = config_.astar_resolution;
  const double dx_world = goal.x - start.x;
  const double dy_world = goal.y - start.y;
  const double xy_length_squared = dx_world * dx_world + dy_world * dy_world;
  if (xy_length_squared < resolution * resolution) {
    return false;
  }
  const int goal_x = static_cast<int>(std::lround(dx_world / resolution));
  const int goal_y = static_cast<int>(std::lround(dy_world / resolution));
  const int margin = static_cast<int>(std::ceil(config_.astar_search_margin / resolution));
  const int min_x = std::min(0, goal_x) - margin;
  const int max_x = std::max(0, goal_x) + margin;
  const int min_y = std::min(0, goal_y) - margin;
  const int max_y = std::max(0, goal_y) + margin;

  const auto pointAt = [&](int x, int y) {
      ScanPoint point{start.x + x * resolution, start.y + y * resolution, start.z};
      const double along = ((point.x - start.x) * dx_world +
        (point.y - start.y) * dy_world) / xy_length_squared;
      const double ratio = std::clamp(along, 0.0, 1.0);
      point.z = start.z + ratio * (goal.z - start.z);
      return point;
    };
  const auto heuristic = [&](int x, int y) {
      return std::hypot(static_cast<double>(goal_x - x), static_cast<double>(goal_y - y));
    };

  std::unordered_map<int64_t, Node> nodes;
  nodes.reserve(static_cast<size_t>((max_x - min_x + 1) * (max_y - min_y + 1)));
  std::priority_queue<QueueEntry> open;
  const int64_t start_key = latticeKey(0, 0);
  nodes[start_key] = Node{0, 0, 0.0, heuristic(0, 0), 0, false, false};
  open.push({nodes[start_key].f, start_key});
  int expansions = 0;
  int64_t reached_key = 0;
  bool reached = false;
  constexpr std::array<std::array<int, 2>, 8> kNeighbors{{
    {{-1, -1}}, {{-1, 0}}, {{-1, 1}}, {{0, -1}},
    {{0, 1}}, {{1, -1}}, {{1, 0}}, {{1, 1}}}};

  while (!open.empty() && expansions++ < config_.astar_max_expansions) {
    const auto entry = open.top();
    open.pop();
    auto current_it = nodes.find(entry.key);
    if (current_it == nodes.end() || current_it->second.closed) {
      continue;
    }
    Node & current = current_it->second;
    current.closed = true;
    if (std::abs(current.x - goal_x) <= 1 && std::abs(current.y - goal_y) <= 1) {
      reached_key = entry.key;
      reached = true;
      break;
    }

    for (const auto & offset : kNeighbors) {
      const int nx = current.x + offset[0];
      const int ny = current.y + offset[1];
      if (nx < min_x || nx > max_x || ny < min_y || ny > max_y) {
        continue;
      }
      const ScanPoint from = pointAt(current.x, current.y);
      const ScanPoint next = pointAt(nx, ny);
      if (!segmentFree(from, next)) {
        continue;
      }
      const double deviation = std::abs(
        (next.x - start.x) * dy_world - (next.y - start.y) * dx_world) /
        std::sqrt(xy_length_squared);
      if (deviation > config_.max_reference_deviation) {
        continue;
      }
      const double step = std::hypot(
        static_cast<double>(offset[0]),
        static_cast<double>(offset[1]));
      const double tentative = current.g + step +
        config_.reference_weight * deviation / resolution * 0.05;
      const int64_t next_key = latticeKey(nx, ny);
      auto next_it = nodes.find(next_key);
      if (next_it == nodes.end() || tentative < next_it->second.g) {
        Node node;
        node.x = nx;
        node.y = ny;
        node.g = tentative;
        node.f = tentative + heuristic(nx, ny);
        node.parent = entry.key;
        node.has_parent = true;
        nodes[next_key] = node;
        open.push({node.f, next_key});
      }
    }
  }
  if (!reached) {
    return false;
  }

  replacement.clear();
  int64_t cursor = reached_key;
  while (true) {
    const Node & node = nodes.at(cursor);
    replacement.push_back(pointAt(node.x, node.y));
    if (!node.has_parent) {
      break;
    }
    cursor = node.parent;
  }
  std::reverse(replacement.begin(), replacement.end());
  if (distance(replacement.back(), goal) > 1e-6) {
    if (!segmentFree(replacement.back(), goal)) {
      return false;
    }
    replacement.push_back(goal);
  }
  return true;
}

void SingleScanPlanner::optimize(Points & points) const
{
  if (points.size() < 3) {
    return;
  }
  const Points reference = points;
  const double required_clearance = footprintClearance() + config_.collision_distance;
  const auto local_bounds = map_->localBounds();
  const auto inside_local_esdf = [&local_bounds](const ScanPoint & point) {
      return point.x >= local_bounds.min_x && point.x <= local_bounds.max_x &&
             point.y >= local_bounds.min_y && point.y <= local_bounds.max_y &&
             point.z >= local_bounds.min_z && point.z <= local_bounds.max_z;
    };
  for (int iteration = 0; iteration < config_.optimization_iterations; ++iteration) {
    Points next = points;
    for (size_t i = 1; i + 1 < points.size(); ++i) {
      const ScanPoint average = scale(add(points[i - 1], points[i + 1]), 0.5);
      ScanPoint gradient = scale(subtract(average, points[i]), config_.smooth_weight);
      gradient = add(
        gradient, scale(
          subtract(reference[i], points[i]), config_.reference_weight));

      double clearance = 0.0;
      geometry_msgs::msg::Vector3 map_gradient;
      if (inside_local_esdf(points[i]) &&
        map_->distanceAndGradientAt(toMessage(points[i]), clearance, map_gradient) &&
        clearance < required_clearance)
      {
        const double penalty = required_clearance - clearance;
        gradient.x += config_.collision_weight * penalty * map_gradient.x;
        gradient.y += config_.collision_weight * penalty * map_gradient.y;
        gradient.z += config_.collision_weight * penalty * map_gradient.z;
      }

      const ScanPoint second_difference = add(
        subtract(points[i - 1], scale(points[i], 2.0)), points[i + 1]);
      gradient = add(gradient, scale(second_difference, config_.feasibility_weight));
      ScanPoint move = scale(gradient, config_.optimization_step_size);
      const double move_norm = norm(move);
      if (move_norm > config_.control_point_distance) {
        move = scale(move, config_.control_point_distance / move_norm);
      }
      ScanPoint candidate = add(points[i], move);
      const ScanPoint deviation = subtract(candidate, reference[i]);
      if (norm(deviation) > config_.max_reference_deviation) {
        candidate = add(
          reference[i], scale(
            deviation, config_.max_reference_deviation / norm(deviation)));
      }
      const double yaw = std::atan2(
        points[i + 1].y - points[i - 1].y,
        points[i + 1].x - points[i - 1].x);
      if (poseFree(candidate, yaw)) {
        next[i] = candidate;
      }
    }
    points.swap(next);
  }
}

SingleScanPlanner::Points SingleScanPlanner::sampleCubicBspline(
  const Points & control_points) const
{
  if (control_points.size() < 3) {
    return control_points;
  }
  Points padded{control_points.front(), control_points.front()};
  padded.insert(padded.end(), control_points.begin(), control_points.end());
  padded.push_back(control_points.back());
  padded.push_back(control_points.back());

  Points result;
  for (size_t i = 0; i + 3 < padded.size(); ++i) {
    const double segment_length = distance(padded[i + 1], padded[i + 2]);
    const int samples = std::max(
      1, static_cast<int>(
        std::ceil(segment_length / config_.sample_distance)));
    for (int sample = 0; sample < samples; ++sample) {
      const double u = static_cast<double>(sample) / samples;
      const double u2 = u * u;
      const double u3 = u2 * u;
      const std::array<double, 4> basis{{
        (1.0 - 3.0 * u + 3.0 * u2 - u3) / 6.0,
        (4.0 - 6.0 * u2 + 3.0 * u3) / 6.0,
        (1.0 + 3.0 * u + 3.0 * u2 - 3.0 * u3) / 6.0,
        u3 / 6.0}};
      ScanPoint point;
      for (size_t j = 0; j < basis.size(); ++j) {
        point = add(point, scale(padded[i + j], basis[j]));
      }
      if (result.empty() || distance(result.back(), point) > 1e-5) {
        result.push_back(point);
      }
    }
  }
  if (result.empty() || distance(result.back(), control_points.back()) > 1e-5) {
    result.push_back(control_points.back());
  }
  result.front() = control_points.front();
  return result;
}

void SingleScanPlanner::parameterize(
  const Points & points, TimedScanTrajectory & trajectory) const
{
  trajectory.points = points;
  trajectory.times.assign(points.size(), 0.0);
  if (points.size() < 2) {
    return;
  }
  std::vector<double> speeds(points.size(), config_.max_velocity);
  speeds.front() = 0.0;
  speeds.back() = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    const double ds = distance(points[i - 1], points[i]);
    speeds[i] = std::min(
      speeds[i], std::sqrt(
        speeds[i - 1] * speeds[i - 1] + 2.0 * config_.max_acceleration * ds));
  }
  for (size_t i = points.size() - 1; i > 0; --i) {
    const double ds = distance(points[i - 1], points[i]);
    speeds[i - 1] = std::min(
      speeds[i - 1], std::sqrt(
        speeds[i] * speeds[i] + 2.0 * config_.max_acceleration * ds));
  }
  for (size_t i = 1; i < points.size(); ++i) {
    const double ds = distance(points[i - 1], points[i]);
    const double denominator = speeds[i - 1] + speeds[i];
    const double dt = denominator > 1e-6 ? 2.0 * ds / denominator : 0.0;
    trajectory.times[i] = trajectory.times[i - 1] + dt;
  }
  trajectory.duration = trajectory.times.back();
}

bool SingleScanPlanner::poseFree(const ScanPoint & point, double yaw) const
{
  return map_->isCollisionFree(makePose(point, yaw), footprint_);
}

bool SingleScanPlanner::segmentFree(const ScanPoint & start, const ScanPoint & goal) const
{
  const double yaw = std::atan2(goal.y - start.y, goal.x - start.x);
  return map_->raycastFree(makePose(start, yaw), makePose(goal, yaw), footprint_);
}

double SingleScanPlanner::footprintClearance() const
{
  using navflex_rog_map::FootprintType;
  if (footprint_.type == FootprintType::DOUBLE_SPHERE) {
    return std::max(footprint_.front_sphere.radius, footprint_.rear_sphere.radius) +
           footprint_.safety_margin;
  }
  if (footprint_.type == FootprintType::BOX) {
    return 0.5 * std::hypot(footprint_.size.x, footprint_.size.y) + footprint_.safety_margin;
  }
  return footprint_.radius + footprint_.safety_margin;
}

}  // namespace navflex_scan_planner
