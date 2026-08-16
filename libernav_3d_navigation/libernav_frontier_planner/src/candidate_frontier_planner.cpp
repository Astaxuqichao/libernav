// Copyright 2026 LiberNav contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "libernav_frontier_planner/candidate_frontier_planner.hpp"

#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace libernav_frontier_planner
{
namespace
{
constexpr uint32_t kSuccess = 0;
constexpr uint32_t kInvalidStart = 101;
constexpr uint32_t kInvalidGoal = 102;
constexpr uint32_t kNoPath = 103;

double pathLength(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (std::size_t index = 1; index < path.poses.size(); ++index) {
    const auto & first = path.poses[index - 1].pose.position;
    const auto & second = path.poses[index].pose.position;
    length += std::sqrt(
      (second.x - first.x) * (second.x - first.x) +
      (second.y - first.y) * (second.y - first.y) +
      (second.z - first.z) * (second.z - first.z));
  }
  return length;
}
}  // namespace

void CandidateFrontierPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
  libernav_rog_map::RogMap::ConstPtr map)
{
  core_.configure(parent, name, std::move(tf), std::move(map));
}

void CandidateFrontierPlanner::cleanup() {core_.cleanup();}
void CandidateFrontierPlanner::activate() {core_.activate();}
void CandidateFrontierPlanner::deactivate() {core_.deactivate();}
bool CandidateFrontierPlanner::cancel() {return core_.cancel();}

uint32_t CandidateFrontierPlanner::makePlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  libernav_rogmap_core::Trajectory3D & trajectory, std::string & message)
{
  trajectory = libernav_rogmap_core::Trajectory3D();
  if (start.header.frame_id.empty()) {
    message = "Invalid start: frame is empty";
    return kInvalidStart;
  }
  if (goal.header.frame_id.empty()) {
    message = "Invalid goal: frame is empty";
    return kInvalidGoal;
  }
  const auto candidates = core_.selectCandidates(start, goal);
  if (candidates.empty()) {
    message = "ROGMap contains no reachable frontier candidate";
    return kNoPath;
  }
  trajectory.path = core_.makeCandidatePath(start, candidates);
  trajectory.max_velocity = core_.nominalVelocity();
  trajectory.duration = pathLength(trajectory.path) / trajectory.max_velocity;
  message = "Selected ROGMap frontier candidates";
  return kSuccess;
}

}  // namespace libernav_frontier_planner

PLUGINLIB_EXPORT_CLASS(
  libernav_frontier_planner::CandidateFrontierPlanner,
  libernav_rogmap_core::GlobalPlanner)
