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

#include "velocity_input_controller/velocity_input_controller.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace velocity_input_controller
{

VelocityInputController::VelocityInputController()
: controller_interface::ChainableControllerInterface()
{
}

controller_interface::InterfaceConfiguration
VelocityInputController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(2);
  command_interfaces_config.names.push_back(params_.command_interface_linear);
  command_interfaces_config.names.push_back(params_.command_interface_angular);

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
VelocityInputController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type VelocityInputController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  auto logger = get_node()->get_logger();

  TwistStampedMsg::SharedPtr command_msg_ptr = nullptr;

  // TODO: Add a bit more logic that checks if multiple topics are being published at the same time
  // and sends a warning if so
  for (auto subscriber : velocity_command_subscribers_) {
    if (!subscriber->timeout(time)) {
      command_msg_ptr = subscriber->get_velocity_command();
      break;
    }
  }

  // No command message received or timeout was reached
  if (command_msg_ptr == nullptr) {
    reference_interfaces_[0] = 0.0;
    reference_interfaces_[1] = 0.0;
  } else if (
    std::isfinite(command_msg_ptr->twist.linear.x) &&
    std::isfinite(command_msg_ptr->twist.angular.z)) {
    reference_interfaces_[0] = command_msg_ptr->twist.linear.x;
    reference_interfaces_[1] = command_msg_ptr->twist.angular.z;
  } else {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger, *get_node()->get_clock(), 1000,
      "Command message contains NaNs. Not updating reference interfaces.");
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type VelocityInputController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  double linear_command = reference_interfaces_[0];
  double angular_command = reference_interfaces_[1];

  const auto linear_result = command_interfaces_[0].set_value(linear_command);
  const auto angular_result = command_interfaces_[1].set_value(angular_command);

  if (!linear_result || !angular_result) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to set the command to one of the command handles!");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn VelocityInputController::on_init()
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

controller_interface::CallbackReturn VelocityInputController::on_configure(
  const rclcpp_lifecycle::State &)
{
  reference_interfaces_.resize(kReferenceInterfacesSize, std::numeric_limits<double>::quiet_NaN());

  // Initialize subscribers
  for (const auto & name : params_.cmd_vel_input_topics) {
    velocity_command_subscribers_.push_back(
      std::make_shared<VelocityCommandSubscriber>(this->get_node(), name, params_.cmd_vel_timeout));
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityInputController::on_activate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityInputController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityInputController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityInputController::on_error(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
VelocityInputController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name() + std::string("/linear"), hardware_interface::HW_IF_VELOCITY,
    &reference_interfaces_[0]));
  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name() + std::string("/angular"), hardware_interface::HW_IF_VELOCITY,
    &reference_interfaces_[1]));

  return reference_interfaces;
}

}  // namespace velocity_input_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  velocity_input_controller::VelocityInputController,
  controller_interface::ChainableControllerInterface)
