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

#include "low_pass_filter/low_pass_filter.hpp"

#include <control_toolbox/low_pass_filter.hpp>
#include <controller_interface/chainable_controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace low_pass_filter
{

LowPassFilter::LowPassFilter() : controller_interface::ChainableControllerInterface() {}

controller_interface::InterfaceConfiguration LowPassFilter::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration LowPassFilter::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : params_.state_interface_names) {
    state_interfaces_config.names.push_back(params_.state_interface_prefix + "/" + name);
  }
  return state_interfaces_config;
}

controller_interface::return_type LowPassFilter::update_reference_from_subscribers(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type LowPassFilter::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (!low_pass_filter_ || !low_pass_filter_->is_configured()) {
    RCLCPP_WARN(get_node()->get_logger(), "Low pass filter is not configured");
    return controller_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    low_pass_filter_->update(
      state_interfaces_[i].get_optional<double>().value(), state_interfaces_values_[i]);
  }

  if (realtime_publisher_ && realtime_publisher_->trylock()) {
    realtime_publisher_->msg_.header.stamp = time;
    realtime_publisher_->msg_.states.interface_names = exported_state_interface_names_;
    realtime_publisher_->msg_.states.values = state_interfaces_values_;
    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn LowPassFilter::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & err) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to load parameters: %s", err.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LowPassFilter::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  if (params_.publish_interface_values) {
    interface_values_publisher_ = get_node()->create_publisher<DynamicInterfaceValuesMsg>(
      "~/interface_values", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<StatePublisher>(interface_values_publisher_);
  }

  low_pass_filter_ = std::make_shared<control_toolbox::LowPassFilter<double>>(
    params_.sampling_frequency, params_.damping_frequency, params_.damping_intensity);
  low_pass_filter_->configure();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LowPassFilter::on_activate(const rclcpp_lifecycle::State &)
{
  // update parameters if they have changed
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(get_node()->get_logger(), "Parameters were updated");

    low_pass_filter_->set_params(
      params_.sampling_frequency, params_.damping_frequency, params_.damping_intensity);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LowPassFilter::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Deactivate the controller
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LowPassFilter::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  const auto state_interfaces_size = params_.state_interface_names.size();
  state_interfaces.reserve(state_interfaces_size);

  state_interfaces_values_.resize(state_interfaces_size, std::numeric_limits<double>::quiet_NaN());

  std::size_t index = 0;
  for (const auto & name : params_.state_interface_names) {
    state_interfaces.push_back(hardware_interface::StateInterface(
      std::string(this->get_node()->get_name()) + "/imu", name, &state_interfaces_values_[index]));
    index++;
  }

  return state_interfaces;
}

}  // namespace low_pass_filter

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  low_pass_filter::LowPassFilter, controller_interface::ChainableControllerInterface)
