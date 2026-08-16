// Copyright 2026 LiberNav contributors

#ifndef LIBERNAV_ROGMAP_CORE__TYPES_HPP_
#define LIBERNAV_ROGMAP_CORE__TYPES_HPP_

#include "nav_msgs/msg/path.hpp"

namespace libernav_rogmap_core
{
struct Trajectory3D {nav_msgs::msg::Path path; double duration{0.0}; double max_velocity{0.0};
  double max_acceleration{0.0};};
}  // namespace libernav_rogmap_core
#endif  // LIBERNAV_ROGMAP_CORE__TYPES_HPP_
