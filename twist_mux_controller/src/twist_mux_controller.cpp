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

#include "twist_mux_controller/twist_mux_controller.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace twist_mux_controller
{

TwistMuxController::TwistMuxController() : controller_interface::ChainableControllerInterface() {}

controller_interface::InterfaceConfiguration TwistMuxController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (params_.holonomic) {
    command_interfaces_config.names.reserve(3);
    command_interfaces_config.names.push_back(params_.command_interface_linear_x);
    command_interfaces_config.names.push_back(params_.command_interface_linear_y);
    command_interfaces_config.names.push_back(params_.command_interface_angular_z);
  } else {
    command_interfaces_config.names.reserve(2);
    command_interfaces_config.names.push_back(params_.command_interface_linear_x);
    command_interfaces_config.names.push_back(params_.command_interface_angular_z);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration TwistMuxController::state_interface_configuration()
  const
{
  std::vector<std::string> conf_names;
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type TwistMuxController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  std::string source = kSourceNotPublished;

  TwistStampedMsg::SharedPtr command_msg_ptr = nullptr;

  // TODO: Add a bit more logic that checks if multiple topics are being published at the same time
  // and sends a warning if so.
  std::uint8_t highest_priority = 0;
  for (auto subscriber : cmd_vel_subscribers_) {
    if (subscriber->get_priority() >= highest_priority && !subscriber->timeout(time)) {
      command_msg_ptr = subscriber->get_velocity_command();
      source = subscriber->get_source_type();
      highest_priority = subscriber->get_priority();
    }
  }

  update_reference_interfaces(command_msg_ptr);

  // Publish the source of the cmd_vel
  if (cmd_vel_source_ != source && realtime_cmd_vel_source_publisher_->trylock()) {
    realtime_cmd_vel_source_publisher_->msg_.data = source;
    realtime_cmd_vel_source_publisher_->unlockAndPublish();
    cmd_vel_source_ = source;
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type TwistMuxController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  auto result = std::vector<bool>();

  if (params_.holonomic) {
    result.push_back(command_interfaces_[0].set_value(reference_interfaces_[0]));
    result.push_back(command_interfaces_[1].set_value(reference_interfaces_[1]));
    result.push_back(command_interfaces_[2].set_value(reference_interfaces_[2]));
  } else {
    result.push_back(command_interfaces_[0].set_value(reference_interfaces_[0]));
    result.push_back(command_interfaces_[1].set_value(reference_interfaces_[1]));
  }

  if (!std::all_of(result.begin(), result.end(), [](bool success) { return success; })) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to set the command to one of the command handles!");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn TwistMuxController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & err) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to load parameters: %s", err.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  rcl_interfaces::msg::ListParametersResult list = get_node()->list_parameters(
    {"cmd_vel_inputs"}, 10);

  for (const auto & prefix : list.prefixes) {
    if (!get_node()->has_parameter(prefix + ".topic")) {
      get_node()->declare_parameter(prefix + ".topic", get_source_from_prefix(prefix) + "/cmd_vel");
    }
    if (!get_node()->has_parameter(prefix + ".timeout")) {
      get_node()->declare_parameter(prefix + ".timeout", 0.5);
    }
    if (!get_node()->has_parameter(prefix + ".priority")) {
      get_node()->declare_parameter(prefix + ".priority", 0);
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TwistMuxController::on_configure(
  const rclcpp_lifecycle::State &)
{
  const auto reference_interfaces_size = params_.holonomic ? 3 : 2;
  reference_interfaces_.resize(reference_interfaces_size, std::numeric_limits<double>::quiet_NaN());

  rcl_interfaces::msg::ListParametersResult list = get_node()->list_parameters(
    {"cmd_vel_inputs"}, 10);

  for (const auto & prefix : list.prefixes) {
    std::string topic;
    double timeout = 0;
    std::uint8_t priority = 0;

    get_node()->get_parameter(prefix + ".topic", topic);
    get_node()->get_parameter(prefix + ".timeout", timeout);
    get_node()->get_parameter(prefix + ".priority", priority);

    RCLCPP_DEBUG(
      get_node()->get_logger(), "Params for prefix: %s: topic: %s, timeout: %f, priority: %d",
      prefix.c_str(), topic.c_str(), timeout, priority);

    // TODO: Add a check if there are no multiple inputs with the same priority

    cmd_vel_subscribers_.push_back(std::make_shared<TwistMsgSubscriber>(
      this->get_node(), topic, get_source_from_prefix(prefix), timeout, priority));
  }

  cmd_vel_source_publisher_ = this->get_node()->create_publisher<StringMsg>(
    "~/source", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  realtime_cmd_vel_source_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<StringMsg>>(cmd_vel_source_publisher_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TwistMuxController::on_activate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TwistMuxController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TwistMuxController::on_cleanup(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TwistMuxController::on_error(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
TwistMuxController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  if (params_.holonomic) {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name() + std::string("/linear/x"), hardware_interface::HW_IF_VELOCITY,
      &reference_interfaces_[0]));
    reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name() + std::string("/linear/y"), hardware_interface::HW_IF_VELOCITY,
      &reference_interfaces_[1]));
    reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name() + std::string("/angular/z"), hardware_interface::HW_IF_VELOCITY,
      &reference_interfaces_[2]));
  } else {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name() + std::string("/linear/x"), hardware_interface::HW_IF_VELOCITY,
      &reference_interfaces_[0]));
    reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name() + std::string("/angular/z"), hardware_interface::HW_IF_VELOCITY,
      &reference_interfaces_[1]));
  }

  return reference_interfaces;
}

void TwistMuxController::update_reference_interfaces(
  const TwistStampedMsg::SharedPtr & command_msg_ptr)
{
  if (command_msg_ptr == nullptr) {
    std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), 0.0);
    return;
  }

  auto commands = std::vector<double>();
  if (params_.holonomic) {
    commands = {
      command_msg_ptr->twist.linear.x, command_msg_ptr->twist.linear.y,
      command_msg_ptr->twist.angular.z};
  } else {
    commands = {command_msg_ptr->twist.linear.x, command_msg_ptr->twist.angular.z};
  }

  if (std::any_of(
        commands.begin(), commands.end(), [](double value) { return !std::isfinite(value); })) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Command message contains NaNs. Not updating reference interfaces.");
    return;
  }

  std::copy(commands.begin(), commands.end(), reference_interfaces_.begin());
}

std::string TwistMuxController::get_source_from_prefix(const std::string & prefix) const
{
  auto pos = prefix.rfind('.');
  if (pos != std::string::npos) {
    return prefix.substr(pos + 1);
  }

  return prefix;
}

}  // namespace twist_mux_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  twist_mux_controller::TwistMuxController, controller_interface::ChainableControllerInterface)
