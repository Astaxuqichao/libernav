// Copyright 2026 LiberNav contributors

#ifndef LIBERNAV_ROGMAP_CORE__GLOBAL_PLANNER_HPP_
#define LIBERNAV_ROGMAP_CORE__GLOBAL_PLANNER_HPP_
#include <cstdint>
#include <memory>
#include <string>
#include "libernav_rog_map/rog_map.hpp"
#include "libernav_rogmap_core/types.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
namespace libernav_rogmap_core
{
class GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<GlobalPlanner>; virtual ~GlobalPlanner() = default;
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    std::shared_ptr<tf2_ros::Buffer>, libernav_rog_map::RogMap::ConstPtr) = 0;
  virtual void cleanup() = 0; virtual void activate() = 0; virtual void deactivate() = 0;
  virtual uint32_t makePlan(
    const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::PoseStamped &,
    Trajectory3D &, std::string & message) = 0;
  virtual bool cancel() {return true;}
};
}  // namespace libernav_rogmap_core
#endif  // LIBERNAV_ROGMAP_CORE__GLOBAL_PLANNER_HPP_
