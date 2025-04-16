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
  command_interfaces_config.names.push_back(
    std::string("drive_controller/linear/") + hardware_interface::HW_IF_VELOCITY);
  command_interfaces_config.names.push_back(
    std::string("pid_controller/low_pass_filter/imu/angular_velocity.z"));

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

  const std::shared_ptr<TwistStampedMsg> command_msg_ptr =
    *(received_velocity_msg_ptr_.readFromRT());

  if (command_msg_ptr == nullptr) {
    RCLCPP_INFO_ONCE(
      logger, "Waiting for command message to be received. This message will only be shown once.");
    return controller_interface::return_type::OK;
  }

  const auto age_of_last_command = time - command_msg_ptr->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (age_of_last_command > cmd_vel_timeout_) {
    reference_interfaces_[0] = 0.0;
    reference_interfaces_[1] = 0.0;
  } else if (
    std::isfinite(command_msg_ptr->twist.linear.x) &&
    std::isfinite(command_msg_ptr->twist.angular.z)) {
    reference_interfaces_[0] = command_msg_ptr->twist.linear.x;
    reference_interfaces_[1] = command_msg_ptr->twist.angular.z;
  } else {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger, *get_node()->get_clock(), cmd_vel_timeout_.seconds() * 1000,
      "Command message contains NaNs. Not updating reference interfaces.");
  }

  previous_update_timestamp_ = time;

  return controller_interface::return_type::OK;
}

controller_interface::return_type VelocityInputController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  double linear_command = reference_interfaces_[0];
  double angular_command = reference_interfaces_[1];

  command_interfaces_[0].set_value(linear_command);
  command_interfaces_[1].set_value(angular_command);

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn VelocityInputController::on_init()
{
  // todo: define parameters
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityInputController::on_configure(
  const rclcpp_lifecycle::State &)
{
  // todo: read parameters

  reference_interfaces_.resize(kReferenceInterfacesSize, std::numeric_limits<double>::quiet_NaN());

  // Initialize subscribers
  velocity_command_subscriber_ = get_node()->create_subscription<TwistStampedMsg>(
    "cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<TwistStampedMsg> msg) -> void {
      if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
        RCLCPP_WARN_ONCE(
          get_node()->get_logger(),
          "Received TwistStamped with zero timestamp, setting it to current "
          "time, this message will only be shown once");
        msg->header.stamp = get_node()->now();
      }

      const auto current_time_diff = get_node()->now() - msg->header.stamp;

      if (
        cmd_vel_timeout_ == rclcpp::Duration::from_seconds(0.0) ||
        current_time_diff < cmd_vel_timeout_) {
        received_velocity_msg_ptr_.writeFromNonRT(msg);
      } else {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Ignoring the received message (timestamp %.10f) because it is older than "
          "the current time by %.10f seconds, which exceeds the allowed timeout (%.4f)",
          rclcpp::Time(msg->header.stamp).seconds(), current_time_diff.seconds(),
          cmd_vel_timeout_.seconds());
      }
    });

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
