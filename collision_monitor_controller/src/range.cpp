// Copyright (c) 2022 Samsung R&D Institute Russia
// Copyright 2025 Husarion sp. z o.o.
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

#include "collision_monitor_controller/range.hpp"

#include <math.h>
#include <cmath>
#include <functional>

#include "tf2/transform_datatypes.h"

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace collision_monitor_controller
{

Range::Range(
  const nav2_util::LifecycleNode::WeakPtr & node, const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string & base_frame_id,
  const std::string & global_frame_id, const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout, const bool base_shift_correction)
: Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id, transform_tolerance,
    source_timeout, base_shift_correction)
{
  RCLCPP_INFO(logger_, "[%s]: Creating Range", source_name_.c_str());
}

Range::~Range()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Range", source_name_.c_str());
  data_sub_.reset();
}

void Range::configure()
{
  Source::configure();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  getParameters(source_topic);

  rclcpp::QoS range_qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<sensor_msgs::msg::Range>(
    source_topic, range_qos, std::bind(&Range::dataCallback, this, std::placeholders::_1));
}

bool Range::getData(const rclcpp::Time & curr_time, std::vector<Point> & data)
{
  const auto latest_data = *received_data_msg_ptr_.readFromRT();
  const auto latest_tf_transform = *latest_tf_transform_ptr_.readFromRT();

  // Ignore data from the source if it is not being published yet or
  // not being published for a long time
  if (latest_data == nullptr || latest_tf_transform == nullptr) {
    return false;
  }
  if (!sourceValid(latest_data->header.stamp, curr_time)) {
    return false;
  }

  // Ignore data, if its range is out of scope of range sensor abilities
  if (latest_data->range < latest_data->min_range || latest_data->range > latest_data->max_range) {
    RCLCPP_DEBUG(
      logger_, "[%s]: Data range %fm is out of {%f..%f} sensor span. Ignoring...",
      source_name_.c_str(), latest_data->range, latest_data->min_range, latest_data->max_range);
    return false;
  }

  // Calculate poses and refill data array
  float angle;
  for (angle = -latest_data->field_of_view / 2; angle < latest_data->field_of_view / 2;
       angle += obstacles_angle_) {
    // Transform point coordinates from source frame -> to base frame
    tf2::Vector3 p_v3_s(
      latest_data->range * std::cos(angle), latest_data->range * std::sin(angle), 0.0);
    tf2::Vector3 p_v3_b = *latest_tf_transform * p_v3_s;

    // Refill data array
    data.push_back({p_v3_b.x(), p_v3_b.y()});
  }

  // Make sure that last (field_of_view / 2) point will be in the data array
  angle = latest_data->field_of_view / 2;

  // Transform point coordinates from source frame -> to base frame
  tf2::Vector3 p_v3_s(
    latest_data->range * std::cos(angle), latest_data->range * std::sin(angle), 0.0);
  tf2::Vector3 p_v3_b = *latest_tf_transform * p_v3_s;

  // Refill data array
  data.push_back({p_v3_b.x(), p_v3_b.y()});

  return true;
}

void Range::getParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  getCommonParameters(source_topic);

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".obstacles_angle", rclcpp::ParameterValue(M_PI / 180));
  obstacles_angle_ = node->get_parameter(source_name_ + ".obstacles_angle").as_double();
}

void Range::dataCallback(sensor_msgs::msg::Range::ConstSharedPtr msg)
{
  received_data_msg_ptr_.writeFromNonRT(msg);

  tf2::Transform tf_transform;
  if (!getTransform(msg->header.stamp, msg->header, tf_transform)) {
    RCLCPP_WARN(
      logger_, "[%s]: Failed to get transform for the latest range data", source_name_.c_str());
    latest_tf_transform_ptr_.writeFromNonRT(nullptr);
    return;
  }

  latest_tf_transform_ptr_.writeFromNonRT(std::make_shared<tf2::Transform>(tf_transform));
}

}  // namespace collision_monitor_controller
