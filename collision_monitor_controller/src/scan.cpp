// Copyright (c) 2022 Samsung R&D Institute Russia
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

#include "collision_monitor_controller/scan.hpp"

#include <cmath>
#include <functional>

#include "tf2/transform_datatypes.h"

#include "nav2_util/robot_utils.hpp"

namespace collision_monitor_controller
{

Scan::Scan(
  const nav2_util::LifecycleNode::WeakPtr & node, const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string & base_frame_id,
  const std::string & global_frame_id, const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout, const bool base_shift_correction)
: Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id, transform_tolerance,
    source_timeout, base_shift_correction)
{
  RCLCPP_INFO(logger_, "[%s]: Creating Scan", source_name_.c_str());
}

Scan::~Scan()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Scan", source_name_.c_str());
  data_sub_.reset();
}

void Scan::configure()
{
  Source::configure();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  // Laser scanner has no own parameters
  getCommonParameters(source_topic);

  rclcpp::QoS scan_qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    source_topic, scan_qos, std::bind(&Scan::dataCallback, this, std::placeholders::_1));
}

bool Scan::getData(const rclcpp::Time & curr_time, std::vector<Point> & data)
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

  // Calculate poses and refill data array
  float angle = latest_data->angle_min;
  for (size_t i = 0; i < latest_data->ranges.size(); i++) {
    if (
      latest_data->ranges[i] >= latest_data->range_min &&
      latest_data->ranges[i] <= latest_data->range_max) {
      // Transform point coordinates from source frame -> to base frame
      tf2::Vector3 p_v3_s(
        latest_data->ranges[i] * std::cos(angle), latest_data->ranges[i] * std::sin(angle), 0.0);
      tf2::Vector3 p_v3_b = *latest_tf_transform * p_v3_s;

      // Refill data array
      data.push_back({p_v3_b.x(), p_v3_b.y()});
    }
    angle += latest_data->angle_increment;
  }
  return true;
}

void Scan::dataCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  received_data_msg_ptr_.writeFromNonRT(msg);

  tf2::Transform tf_transform;
  if (!getTransform(msg->header.stamp, msg->header, tf_transform)) {
    RCLCPP_WARN(
      logger_, "[%s]: Failed to get transform for the latest scan data", source_name_.c_str());
    received_data_msg_ptr_.writeFromNonRT(nullptr);
    return;
  }

  latest_tf_transform_ptr_.writeFromNonRT(std::make_shared<tf2::Transform>(tf_transform));
}

}  // namespace collision_monitor_controller
