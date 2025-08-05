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

#ifndef TWIST_MUX_CONTROLLER_TWIST_MSG_SUBSCRIBER
#define TWIST_MUX_CONTROLLER_TWIST_MSG_SUBSCRIBER

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>

namespace twist_mux_controller
{

using TwistStampedMsg = geometry_msgs::msg::TwistStamped;

class TwistMsgSubscriber
{
public:
  TwistMsgSubscriber(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string & topic_name,
    const std::string & source_type, const float timeout = 0.5, const std::uint8_t priority = 0)
  : node_(std::move(node)),
    source_type_(source_type),
    cmd_vel_timeout_(rclcpp::Duration(std::chrono::milliseconds(static_cast<int>(timeout * 1000)))),
    priority_(priority)
  {
    if (auto node = node_.lock()) {
      subscriber_ = node->create_subscription<TwistStampedMsg>(
        topic_name, rclcpp::SystemDefaultsQoS(),
        std::bind(&TwistMsgSubscriber::velocity_command_callback, this, std::placeholders::_1));
    }
  }

  bool timeout(const rclcpp::Time & time)
  {
    const auto msg_ptr = received_msg_ptr_.readFromRT()->get();

    if (msg_ptr == nullptr) {
      return true;
    }

    return (time - msg_ptr->header.stamp) >= cmd_vel_timeout_;
  }

  TwistStampedMsg::SharedPtr get_velocity_command() { return *(received_msg_ptr_.readFromRT()); }

  std::string get_source_type() const { return source_type_; }

  std::uint8_t get_priority() const { return priority_; }

protected:
  void velocity_command_callback(const TwistStampedMsg::SharedPtr msg)
  {
    if (auto node = node_.lock()) {
      if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
        RCLCPP_WARN_ONCE(
          node->get_logger(),
          "Received TwistStamped with zero timestamp, setting it to current "
          "time, this message will only be shown once");
        msg->header.stamp = node->now();
      }

      const auto current_time_diff = node->now() - msg->header.stamp;

      if (
        cmd_vel_timeout_ != rclcpp::Duration::from_seconds(0.0) &&
        current_time_diff >= cmd_vel_timeout_) {
        RCLCPP_WARN(
          node->get_logger(),
          "Ignoring the received message (timestamp %.10f) because it is older than "
          "the current time by %.10f seconds, which exceeds the allowed timeout (%.4f)",
          rclcpp::Time(msg->header.stamp).seconds(), current_time_diff.seconds(),
          cmd_vel_timeout_.seconds());
        return;
      }

      received_msg_ptr_.writeFromNonRT(msg);
    }
  }

  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  const std::string source_type_;
  const rclcpp::Duration cmd_vel_timeout_;
  const std::uint8_t priority_;

  rclcpp::Subscription<TwistStampedMsg>::SharedPtr subscriber_;
  realtime_tools::RealtimeBuffer<TwistStampedMsg::SharedPtr> received_msg_ptr_{nullptr};
};

}  // namespace twist_mux_controller

#endif  // TWIST_MUX_CONTROLLER_TWIST_MSG_SUBSCRIBER
