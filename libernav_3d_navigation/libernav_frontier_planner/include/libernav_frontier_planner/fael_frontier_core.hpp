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

#ifndef LIBERNAV_FRONTIER_PLANNER__FAEL_FRONTIER_CORE_HPP_
#define LIBERNAV_FRONTIER_PLANNER__FAEL_FRONTIER_CORE_HPP_

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "libernav_rog_map/rog_map.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace libernav_frontier_planner
{

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};

  double distanceXY(const Point3 & other) const;
  double distance(const Point3 & other) const;
};

struct Candidate
{
  Point3 point;
  std::vector<Point3> frontiers;
  double score{0.0};
};

class FaelFrontierCore
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    libernav_rog_map::RogMap::ConstPtr map);
  void activate();
  void deactivate();
  void cleanup();
  bool cancel();

  std::vector<Candidate> selectCandidates(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & requested_goal);
  nav_msgs::msg::Path makeCandidatePath(
    const geometry_msgs::msg::PoseStamped & start,
    const std::vector<Candidate> & candidates) const;
  nav_msgs::msg::Path makeAStarPath(
    const geometry_msgs::msg::PoseStamped & start,
    const Candidate & candidate);
  void publishSelection(
    const geometry_msgs::msg::PoseStamped & start,
    const std::vector<Candidate> & candidates,
    const Candidate & selected) const;
  double nominalVelocity() const {return nominal_velocity_;}

private:
  bool isFree(const Point3 & point, double radius) const;
  bool isVisible(const Point3 & from, const Point3 & to, double radius) const;
  std::vector<Point3> findFrontiers(const Point3 & current) const;
  std::vector<Candidate> buildCandidates(
    const Point3 & current, const Point3 & requested_goal,
    const std::vector<Point3> & frontiers) const;
  void publishCandidates(
    const std::vector<Candidate> & candidates, const Candidate * selected) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  libernav_rog_map::RogMap::ConstPtr map_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    candidate_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    selected_candidate_pub_;

  std::string name_;
  std::string frame_id_{"map"};
  std::string visualization_topic_prefix_{"frontier_exploration"};
  double local_range_{12.0};
  double scan_resolution_{0.4};
  double scan_z_min_{-0.2};
  double scan_z_max_{1.0};
  double sample_dist_{1.0};
  double candidate_separation_{1.0};
  double robot_clear_radius_{0.3};
  double min_candidate_dist_{0.5};
  double frontier_gain_{100.0};
  double distance_weight_{0.1};
  double goal_distance_weight_{0.0};
  int min_frontier_cells_{2};
  int max_candidate_count_{10};
  double planning_resolution_{0.3};
  int max_astar_expansions_{100000};
  double nominal_velocity_{0.5};
  std::atomic_bool cancel_requested_{false};
};

}  // namespace libernav_frontier_planner

#endif  // LIBERNAV_FRONTIER_PLANNER__FAEL_FRONTIER_CORE_HPP_
