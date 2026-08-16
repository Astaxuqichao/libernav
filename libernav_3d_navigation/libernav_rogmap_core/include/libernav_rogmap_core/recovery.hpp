// Copyright 2026 LiberNav contributors

#ifndef LIBERNAV_ROGMAP_CORE__RECOVERY_HPP_
#define LIBERNAV_ROGMAP_CORE__RECOVERY_HPP_
#include <cstdint>
#include <memory>
#include <string>
#include "libernav_rog_map/rog_map.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
namespace libernav_rogmap_core
{
class Recovery
{
public:
  using Ptr = std::shared_ptr<Recovery>; virtual ~Recovery() = default;
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    std::shared_ptr<tf2_ros::Buffer>, libernav_rog_map::RogMap::ConstPtr) = 0;
  virtual void cleanup() = 0; virtual void activate() = 0; virtual void deactivate() = 0;
  virtual uint32_t runBehavior(std::string & message) = 0;
  virtual void stop() {}
};
}  // namespace libernav_rogmap_core
#endif  // LIBERNAV_ROGMAP_CORE__RECOVERY_HPP_
