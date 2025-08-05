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

#ifndef TWIST_MUX_CONTROLLER_TWIST_MUX_CONTROLLER
#define TWIST_MUX_CONTROLLER_TWIST_MUX_CONTROLLER

#include <memory>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include "twist_mux_controller/twist_mux_controller_parameters.hpp"
#include "twist_mux_controller/twist_msg_subscriber.hpp"

namespace twist_mux_controller
{

using StringMsg = std_msgs::msg::String;
using TwistStampedMsg = geometry_msgs::msg::TwistStamped;

class TwistMuxController : public controller_interface::ChainableControllerInterface
{
public:
  TwistMuxController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Chainable controller replaces update() with the following two functions
  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp::NodeOptions define_custom_node_options() const
  {
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
    return node_options;
  }

protected:
  void update_reference_interfaces(const TwistStampedMsg::SharedPtr & command_msg_ptr);
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  std::string get_source_from_prefix(const std::string & prefix) const;

  static constexpr char kSourceNotPublished[] = "not_published";

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::string cmd_vel_source_;

  std::vector<std::shared_ptr<TwistMsgSubscriber>> cmd_vel_subscribers_;
  rclcpp::Publisher<StringMsg>::SharedPtr cmd_vel_source_publisher_;
  realtime_tools::RealtimePublisher<StringMsg>::SharedPtr realtime_cmd_vel_source_publisher_;

private:
  static constexpr uint kReferenceInterfacesSize = 2;
};

}  // namespace twist_mux_controller

#endif  // TWIST_MUX_CONTROLLER_TWIST_MUX_CONTROLLER
