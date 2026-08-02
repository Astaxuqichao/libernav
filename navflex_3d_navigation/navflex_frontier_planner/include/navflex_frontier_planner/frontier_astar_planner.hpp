// Copyright 2026 Navflex contributors
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

#ifndef NAVFLEX_FRONTIER_PLANNER__FRONTIER_ASTAR_PLANNER_HPP_
#define NAVFLEX_FRONTIER_PLANNER__FRONTIER_ASTAR_PLANNER_HPP_

#include <memory>
#include <string>

#include "navflex_frontier_planner/fael_frontier_core.hpp"
#include "navflex_rogmap_core/global_planner.hpp"

namespace navflex_frontier_planner
{

class FrontierAStarPlanner : public navflex_rogmap_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    navflex_rog_map::RogMap::ConstPtr map) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  uint32_t makePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    navflex_rogmap_core::Trajectory3D & trajectory,
    std::string & message) override;
  bool cancel() override;

private:
  FaelFrontierCore core_;
};

}  // namespace navflex_frontier_planner

#endif  // NAVFLEX_FRONTIER_PLANNER__FRONTIER_ASTAR_PLANNER_HPP_
