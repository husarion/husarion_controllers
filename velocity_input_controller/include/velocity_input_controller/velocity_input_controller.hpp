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

#ifndef VELOCITY_INPUT_CONTROLLER_VELOCITY_INPUT_CONTROLLER
#define VELOCITY_INPUT_CONTROLLER_VELOCITY_INPUT_CONTROLLER

#include <memory>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>

#include "velocity_input_controller/velocity_command_subscriber.hpp"
#include "velocity_input_controller/velocity_input_controller_parameters.hpp"

namespace velocity_input_controller
{

using TwistStampedMsg = geometry_msgs::msg::TwistStamped;

class VelocityInputController : public controller_interface::ChainableControllerInterface
{
public:
  VelocityInputController();

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

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::vector<std::shared_ptr<VelocityCommandSubscriber>> velocity_command_subscribers_;
private:
  static constexpr uint kReferenceInterfacesSize = 2;
};

}  // namespace velocity_input_controller

#endif  // VELOCITY_INPUT_CONTROLLER_VELOCITY_INPUT_CONTROLLER
