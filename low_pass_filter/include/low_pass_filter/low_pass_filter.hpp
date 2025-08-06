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

#ifndef LOW_PASS_FILTER_LOW_PASS_FILTER
#define LOW_PASS_FILTER_LOW_PASS_FILTER

#include <control_toolbox/low_pass_filter.hpp>
#include <controller_interface/chainable_controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <control_msgs/msg/dynamic_interface_values.hpp>

#include "low_pass_filter/low_pass_filter_parameters.hpp"

namespace low_pass_filter
{

using DynamicInterfaceValuesMsg = control_msgs::msg::DynamicInterfaceValues;

class LowPassFilter : public controller_interface::ChainableControllerInterface
{
public:
  LowPassFilter();

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

protected:
  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  using StatePublisher = realtime_tools::RealtimePublisher<DynamicInterfaceValuesMsg>;
  rclcpp::Publisher<DynamicInterfaceValuesMsg>::SharedPtr interface_values_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;

  std::shared_ptr<control_toolbox::LowPassFilter<double>> low_pass_filter_;
};

}  // namespace low_pass_filter

#endif  // LOW_PASS_FILTER_LOW_PASS_FILTER
