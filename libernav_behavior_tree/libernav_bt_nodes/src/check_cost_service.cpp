#include "libernav_bt_nodes/check_cost_service.hpp"

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace libernav_bt_nodes
{

CheckPointService::CheckPointService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtServiceNode<nav2_msgs::srv::CheckPoint>(
    service_node_name, conf, "/costmap/check_point_cost")
{
}

void CheckPointService::on_tick()
{
  getInput("point", request_->point);
  getInput("costmap", request_->costmap);
}

BT::NodeStatus CheckPointService::on_completion(
  std::shared_ptr<nav2_msgs::srv::CheckPoint::Response> response)
{
  setOutput("state", response->state);
  setOutput("cost", response->cost);
  return BT::NodeStatus::SUCCESS;
}

CheckPoseService::CheckPoseService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtServiceNode<nav2_msgs::srv::CheckPose>(
    service_node_name, conf, "/costmap/check_pose_cost")
{
}

void CheckPoseService::on_tick()
{
  getInput("pose", request_->pose);
  getInput("safety_dist", request_->safety_dist);
  getInput("lethal_cost_mult", request_->lethal_cost_mult);
  getInput("inscrib_cost_mult", request_->inscrib_cost_mult);
  getInput("unknown_cost_mult", request_->unknown_cost_mult);
  getInput("costmap", request_->costmap);
  getInput("current_pose", request_->current_pose);
}

BT::NodeStatus CheckPoseService::on_completion(
  std::shared_ptr<nav2_msgs::srv::CheckPose::Response> response)
{
  setOutput("state", response->state);
  setOutput("cost", response->cost);
  return BT::NodeStatus::SUCCESS;
}

CheckPathService::CheckPathService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtServiceNode<nav2_msgs::srv::CheckPath>(
    service_node_name, conf, "/costmap/check_path_cost")
{
}

void CheckPathService::on_tick()
{
  getInput("path", request_->path);
  getInput("safety_dist", request_->safety_dist);
  getInput("lethal_cost_mult", request_->lethal_cost_mult);
  getInput("inscrib_cost_mult", request_->inscrib_cost_mult);
  getInput("unknown_cost_mult", request_->unknown_cost_mult);
  getInput("costmap", request_->costmap);
  getInput("return_on", request_->return_on);
  getInput("skip_poses", request_->skip_poses);
  getInput("path_cells_only", request_->path_cells_only);
}

BT::NodeStatus CheckPathService::on_completion(
  std::shared_ptr<nav2_msgs::srv::CheckPath::Response> response)
{
  setOutput("last_checked", response->last_checked);
  setOutput("state", response->state);
  setOutput("cost", response->cost);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace libernav_bt_nodes

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<libernav_bt_nodes::CheckPointService>("LiberNavCheckPointService");
  factory.registerNodeType<libernav_bt_nodes::CheckPoseService>("LiberNavCheckPoseService");
  factory.registerNodeType<libernav_bt_nodes::CheckPathService>("LiberNavCheckPathService");
}
