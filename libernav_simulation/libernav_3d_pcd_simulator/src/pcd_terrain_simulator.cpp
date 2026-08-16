// Copyright 2026 LiberNav contributors
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/common/point_tests.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace libernav_3d_pcd_simulator
{

class PcdTerrainSimulator : public rclcpp::Node
{
public:
  PcdTerrainSimulator()
  : Node("pcd_terrain_simulator", rclcpp::NodeOptions().enable_rosout(false))
  {
    pcd_file_ = declare_parameter<std::string>("pcd_file", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    map_topic_ = declare_parameter<std::string>("map_topic", "/pcd_map");
    update_rate_ = declare_parameter<double>("update_rate", 50.0);
    cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
    ground_clearance_ = declare_parameter<double>("ground_clearance", 0.5);
    ground_grid_resolution_ = declare_parameter<double>("ground_grid_resolution", 0.2);
    ground_search_radius_ = declare_parameter<double>("ground_search_radius", 0.35);
    max_ground_step_ = declare_parameter<double>("max_ground_step", 0.35);
    start_x_ = declare_parameter<double>("start_x", 0.0);
    start_y_ = declare_parameter<double>("start_y", 0.0);
    start_yaw_ = declare_parameter<double>("start_yaw", 0.0);
    publish_clock_ = declare_parameter<bool>("publish_clock", true);

    validateParameters();
    loadTerrain();
    x_ = start_x_;
    y_ = start_y_;
    yaw_ = start_yaw_;
    if (!lookupGroundHeight(x_, y_, std::numeric_limits<double>::quiet_NaN(), ground_z_)) {
      throw std::runtime_error("No PCD ground point found at the configured start_x/start_y");
    }
    z_ = ground_z_ + ground_clearance_;

    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      map_topic_, rclcpp::QoS(1).transient_local());
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
    if (publish_clock_) {
      clock_publisher_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 20);
    }
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 20,
      std::bind(&PcdTerrainSimulator::commandCallback, this, std::placeholders::_1));
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_transform_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    publishMapToOdomTransform();
    publishTerrain();

    const auto period = std::chrono::duration<double>(1.0 / update_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PcdTerrainSimulator::update, this));
    sim_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_command_time_ = sim_time_;

    RCLCPP_INFO(
      get_logger(),
      "Loaded %zu PCD points. Start pose=(%.2f, %.2f, %.2f), clearance=%.2f m",
      terrain_.size(), x_, y_, z_, ground_clearance_);
  }

private:
  struct GroundPoint
  {
    float x;
    float y;
    float z;
  };

  static int64_t gridKey(int x, int y)
  {
    return (static_cast<int64_t>(x) << 32) ^ static_cast<uint32_t>(y);
  }

  void validateParameters() const
  {
    if (pcd_file_.empty()) {
      throw std::invalid_argument("pcd_file is required");
    }
    if (update_rate_ <= 0.0 || cmd_vel_timeout_ < 0.0 || ground_clearance_ < 0.0 ||
      ground_grid_resolution_ <= 0.0 || ground_search_radius_ <= 0.0 || max_ground_step_ < 0.0)
    {
      throw std::invalid_argument("Invalid PCD terrain simulator numeric parameter");
    }
  }

  void loadTerrain()
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile(pcd_file_, cloud) < 0) {
      throw std::runtime_error("Cannot load PCD file: " + pcd_file_);
    }
    terrain_.reserve(cloud.size());
    for (const auto & point : cloud.points) {
      if (!pcl::isFinite(point)) {
        continue;
      }
      const size_t index = terrain_.size();
      terrain_.push_back({point.x, point.y, point.z});
      const int cell_x = static_cast<int>(std::floor(point.x / ground_grid_resolution_));
      const int cell_y = static_cast<int>(std::floor(point.y / ground_grid_resolution_));
      terrain_grid_[gridKey(cell_x, cell_y)].push_back(index);
    }
    if (terrain_.empty()) {
      throw std::runtime_error("PCD contains no finite XYZ points: " + pcd_file_);
    }
  }

  bool lookupGroundHeight(double x, double y, double reference_z, double & height) const
  {
    const int center_x = static_cast<int>(std::floor(x / ground_grid_resolution_));
    const int center_y = static_cast<int>(std::floor(y / ground_grid_resolution_));
    const int range = static_cast<int>(std::ceil(ground_search_radius_ / ground_grid_resolution_));
    const double radius_squared = ground_search_radius_ * ground_search_radius_;
    const bool use_reference = std::isfinite(reference_z);
    double best_score = std::numeric_limits<double>::infinity();
    bool found = false;
    for (int dx = -range; dx <= range; ++dx) {
      for (int dy = -range; dy <= range; ++dy) {
        const auto iterator = terrain_grid_.find(gridKey(center_x + dx, center_y + dy));
        if (iterator == terrain_grid_.end()) {
          continue;
        }
        for (const size_t index : iterator->second) {
          const GroundPoint & point = terrain_[index];
          const double point_dx = point.x - x;
          const double point_dy = point.y - y;
          const double horizontal_squared = point_dx * point_dx + point_dy * point_dy;
          if (horizontal_squared > radius_squared) {
            continue;
          }
          const double vertical_delta = use_reference ? std::abs(point.z - reference_z) : 0.0;
          if (use_reference && vertical_delta > max_ground_step_) {
            continue;
          }
          // Prefer the spatially closest support point, then preserve floor continuity.
          const double score = horizontal_squared + 0.04 * vertical_delta * vertical_delta;
          if (score < best_score) {
            best_score = score;
            height = point.z;
            found = true;
          }
        }
      }
    }
    return found;
  }

  void commandCallback(const geometry_msgs::msg::Twist::SharedPtr command)
  {
    velocity_x_ = command->linear.x;
    velocity_y_ = command->linear.y;
    yaw_velocity_ = command->angular.z;
    last_command_time_ = sim_time_;
  }

  void update()
  {
    const double dt = 1.0 / update_rate_;
    sim_time_ += rclcpp::Duration::from_seconds(dt);
    if (clock_publisher_) {
      rosgraph_msgs::msg::Clock clock;
      clock.clock = sim_time_;
      clock_publisher_->publish(clock);
    }

    if ((sim_time_ - last_command_time_).seconds() > cmd_vel_timeout_) {
      velocity_x_ = 0.0;
      velocity_y_ = 0.0;
      yaw_velocity_ = 0.0;
    }

    const double cosine = std::cos(yaw_);
    const double sine = std::sin(yaw_);
    const double next_x = x_ + (velocity_x_ * cosine - velocity_y_ * sine) * dt;
    const double next_y = y_ + (velocity_x_ * sine + velocity_y_ * cosine) * dt;
    double next_ground_z = ground_z_;
    if (lookupGroundHeight(next_x, next_y, ground_z_, next_ground_z)) {
      x_ = next_x;
      y_ = next_y;
      ground_z_ = next_ground_z;
    } else if (velocity_x_ != 0.0 || velocity_y_ != 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No continuous PCD ground support at requested position; holding current pose");
    }
    const double previous_z = z_;
    z_ = ground_z_ + ground_clearance_;
    yaw_ = std::atan2(std::sin(yaw_ + yaw_velocity_ * dt), std::cos(yaw_ + yaw_velocity_ * dt));
    publishOdometry(dt, (z_ - previous_z) / dt);
  }

  void publishMapToOdomTransform()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.rotation.w = 1.0;
    static_transform_broadcaster_->sendTransform(transform);
  }

  void publishTerrain()
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points.reserve(terrain_.size());
    for (const GroundPoint & point : terrain_) {
      cloud.points.emplace_back(point.x, point.y, point.z);
    }
    cloud.width = static_cast<uint32_t>(cloud.points.size());
    cloud.height = 1;
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = sim_time_;
    map_publisher_->publish(message);
  }

  void publishOdometry(double, double vertical_velocity)
  {
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw_);
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = sim_time_;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = x_;
    odometry.pose.pose.position.y = y_;
    odometry.pose.pose.position.z = z_;
    odometry.pose.pose.orientation.x = orientation.x();
    odometry.pose.pose.orientation.y = orientation.y();
    odometry.pose.pose.orientation.z = orientation.z();
    odometry.pose.pose.orientation.w = orientation.w();
    odometry.twist.twist.linear.x = velocity_x_;
    odometry.twist.twist.linear.y = velocity_y_;
    odometry.twist.twist.linear.z = vertical_velocity;
    odometry.twist.twist.angular.z = yaw_velocity_;
    odom_publisher_->publish(odometry);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = z_;
    transform.transform.rotation = odometry.pose.pose.orientation;
    transform_broadcaster_->sendTransform(transform);
  }

  std::string pcd_file_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string cmd_vel_topic_;
  std::string odom_topic_;
  std::string map_topic_;
  double update_rate_{50.0};
  double cmd_vel_timeout_{0.5};
  double ground_clearance_{0.5};
  double ground_grid_resolution_{0.2};
  double ground_search_radius_{0.35};
  double max_ground_step_{0.35};
  double start_x_{0.0};
  double start_y_{0.0};
  double start_yaw_{0.0};
  bool publish_clock_{true};
  std::vector<GroundPoint> terrain_;
  std::unordered_map<int64_t, std::vector<size_t>> terrain_grid_;
  double x_{0.0};
  double y_{0.0};
  double z_{0.0};
  double yaw_{0.0};
  double ground_z_{0.0};
  double velocity_x_{0.0};
  double velocity_y_{0.0};
  double yaw_velocity_{0.0};
  rclcpp::Time sim_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;
};

}  // namespace libernav_3d_pcd_simulator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<libernav_3d_pcd_simulator::PcdTerrainSimulator>());
  rclcpp::shutdown();
  return 0;
}
