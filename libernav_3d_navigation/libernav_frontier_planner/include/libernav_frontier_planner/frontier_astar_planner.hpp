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

#ifndef LIBERNAV_FRONTIER_PLANNER__FRONTIER_ASTAR_PLANNER_HPP_
#define LIBERNAV_FRONTIER_PLANNER__FRONTIER_ASTAR_PLANNER_HPP_

#include <memory>
#include <string>

#include "libernav_frontier_planner/fael_frontier_core.hpp"
#include "libernav_rogmap_core/global_planner.hpp"

namespace libernav_frontier_planner
{

class FrontierAStarPlanner : public libernav_rogmap_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    libernav_rog_map::RogMap::ConstPtr map) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  uint32_t makePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    libernav_rogmap_core::Trajectory3D & trajectory,
    std::string & message) override;
  bool cancel() override;

private:
  FaelFrontierCore core_;
};

}  // namespace libernav_frontier_planner

#endif  // LIBERNAV_FRONTIER_PLANNER__FRONTIER_ASTAR_PLANNER_HPP_
