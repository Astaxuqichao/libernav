// Copyright 2026 LiberNav contributors
// SPDX-License-Identifier: Apache-2.0

#include "libernav_rviz_plugins/goal_3d_panel.hpp"

#include <cmath>
#include <string>

#include <QDoubleValidator>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace libernav_rviz_plugins
{
namespace
{
QDoubleSpinBox * makeCoordinateInput(QWidget * parent)
{
  auto * input = new QDoubleSpinBox(parent);
  input->setRange(-10000.0, 10000.0);
  input->setDecimals(3);
  input->setSingleStep(0.1);
  return input;
}
}  // namespace

Goal3DPanel::Goal3DPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  auto * form = new QFormLayout();

  frame_input_ = new QLineEdit("map", this);
  topic_input_ = new QLineEdit("/goal_pose", this);
  x_input_ = makeCoordinateInput(this);
  y_input_ = makeCoordinateInput(this);
  z_input_ = makeCoordinateInput(this);
  z_input_->setValue(0.5);
  yaw_input_ = makeCoordinateInput(this);
  yaw_input_->setRange(-2.0 * M_PI, 2.0 * M_PI);
  yaw_input_->setSuffix(" rad");

  form->addRow("Frame", frame_input_);
  form->addRow("Topic", topic_input_);
  form->addRow("X (m)", x_input_);
  form->addRow("Y (m)", y_input_);
  form->addRow("Z (m)", z_input_);
  form->addRow("Yaw", yaw_input_);
  layout->addLayout(form);

  auto * button_row = new QHBoxLayout();
  auto * send_button = new QPushButton("Send 3D Goal", this);
  status_label_ = new QLabel("Ready", this);
  button_row->addWidget(send_button);
  button_row->addWidget(status_label_, 1);
  layout->addLayout(button_row);

  connect(send_button, &QPushButton::clicked, this, &Goal3DPanel::sendGoal);
  connect(topic_input_, &QLineEdit::editingFinished, this, &Goal3DPanel::updatePublisher);
}

void Goal3DPanel::onInitialize()
{
  const auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    status_label_->setText("RViz ROS node unavailable");
    return;
  }
  node_ = node_abstraction->get_raw_node();
  updatePublisher();
}

void Goal3DPanel::loadSpinBox(
  const rviz_common::Config & config, const QString & key, QDoubleSpinBox * spin_box)
{
  float value;
  if (config.mapGetFloat(key, &value)) {
    spin_box->setValue(value);
  }
}

void Goal3DPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString value;
  if (config.mapGetString("Frame", &value)) {
    frame_input_->setText(value);
  }
  if (config.mapGetString("Topic", &value)) {
    topic_input_->setText(value);
  }
  loadSpinBox(config, "X", x_input_);
  loadSpinBox(config, "Y", y_input_);
  loadSpinBox(config, "Z", z_input_);
  loadSpinBox(config, "Yaw", yaw_input_);
  updatePublisher();
}

void Goal3DPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Frame", frame_input_->text());
  config.mapSetValue("Topic", topic_input_->text());
  config.mapSetValue("X", x_input_->value());
  config.mapSetValue("Y", y_input_->value());
  config.mapSetValue("Z", z_input_->value());
  config.mapSetValue("Yaw", yaw_input_->value());
}

void Goal3DPanel::updatePublisher()
{
  if (!node_) {
    return;
  }
  const std::string topic = topic_input_->text().trimmed().toStdString();
  if (topic.empty()) {
    publisher_.reset();
    publisher_topic_.clear();
    status_label_->setText("Topic is required");
    return;
  }
  if (publisher_ && topic == publisher_topic_) {
    return;
  }
  publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
  publisher_topic_ = topic;
  status_label_->setText("Ready");
}

void Goal3DPanel::sendGoal()
{
  updatePublisher();
  const std::string frame = frame_input_->text().trimmed().toStdString();
  if (!publisher_ || frame.empty()) {
    status_label_->setText(frame.empty() ? "Frame is required" : "Publisher unavailable");
    return;
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = frame;
  goal.header.stamp = node_->now();
  goal.pose.position.x = x_input_->value();
  goal.pose.position.y = y_input_->value();
  goal.pose.position.z = z_input_->value();
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw_input_->value());
  goal.pose.orientation = tf2::toMsg(orientation);
  publisher_->publish(goal);
  status_label_->setText("3D goal sent");
}

}  // namespace libernav_rviz_plugins

PLUGINLIB_EXPORT_CLASS(libernav_rviz_plugins::Goal3DPanel, rviz_common::Panel)
