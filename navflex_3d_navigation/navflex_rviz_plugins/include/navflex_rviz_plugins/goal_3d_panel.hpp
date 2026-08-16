// Copyright 2026 Navflex contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef NAVFLEX_RVIZ_PLUGINS__GOAL_3D_PANEL_HPP_
#define NAVFLEX_RVIZ_PLUGINS__GOAL_3D_PANEL_HPP_

#include <memory>
#include <string>

#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

namespace navflex_rviz_plugins
{

class Goal3DPanel : public rviz_common::Panel
{
public:
  explicit Goal3DPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  void sendGoal();
  void updatePublisher();
  static void loadSpinBox(
    const rviz_common::Config & config, const QString & key, QDoubleSpinBox * spin_box);

  QLineEdit * frame_input_{nullptr};
  QLineEdit * topic_input_{nullptr};
  QDoubleSpinBox * x_input_{nullptr};
  QDoubleSpinBox * y_input_{nullptr};
  QDoubleSpinBox * z_input_{nullptr};
  QDoubleSpinBox * yaw_input_{nullptr};
  QLabel * status_label_{nullptr};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  std::string publisher_topic_;
};

}  // namespace navflex_rviz_plugins

#endif  // NAVFLEX_RVIZ_PLUGINS__GOAL_3D_PANEL_HPP_
