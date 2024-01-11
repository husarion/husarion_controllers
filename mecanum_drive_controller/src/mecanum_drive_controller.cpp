// Copyright 2020 PAL Robotics S.L.
// Copyright 2022 Husarion
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

/*
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

// Copied and adapted from diff_drive_controller (https://github.com/ros-controls/ros2_controllers)
// Based on: https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html
// Author: Maciej Stępień

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "mecanum_drive_controller/mecanum_drive_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace mecanum_drive_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

MecanumDriveController::MecanumDriveController() : controller_interface::ControllerInterface()
{
}

const char* MecanumDriveController::feedback_type() const
{
  return params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

controller_interface::CallbackReturn MecanumDriveController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration MecanumDriveController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  conf_names.push_back(front_left_wheel_name_ + "/" + HW_IF_VELOCITY);
  conf_names.push_back(front_right_wheel_name_ + "/" + HW_IF_VELOCITY);
  conf_names.push_back(rear_left_wheel_name_ + "/" + HW_IF_VELOCITY);
  conf_names.push_back(rear_right_wheel_name_ + "/" + HW_IF_VELOCITY);
  return { interface_configuration_type::INDIVIDUAL, conf_names };
}

InterfaceConfiguration MecanumDriveController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  conf_names.push_back(front_left_wheel_name_ + "/" + feedback_type());
  conf_names.push_back(front_right_wheel_name_ + "/" + feedback_type());
  conf_names.push_back(rear_left_wheel_name_ + "/" + feedback_type());
  conf_names.push_back(rear_right_wheel_name_ + "/" + feedback_type());
  return { interface_configuration_type::INDIVIDUAL, conf_names };
}

controller_interface::return_type MecanumDriveController::update(const rclcpp::Time& time,
                                                                 const rclcpp::Duration& period)
{
  auto logger = get_node()->get_logger();
  if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  std::shared_ptr<Twist> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto age_of_last_command = time - last_command_msg->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (age_of_last_command > cmd_vel_timeout_)
  {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.linear.y = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }

  // command may be limited further by SpeedLimit,
  // without affecting the stored twist command
  Twist command = *last_command_msg;
  double& linear_command_x = command.twist.linear.x;
  double& linear_command_y = command.twist.linear.y;
  double& angular_command = command.twist.angular.z;

  previous_update_timestamp_ = time;

  // Apply (possibly new) multipliers:
  const double wheel_separation_x = params_.wheel_separation_x_multiplier * params_.wheel_separation_x;
  const double wheel_separation_y = params_.wheel_separation_y_multiplier * params_.wheel_separation_y;
  const double wheel_radius = params_.wheel_radius_multiplier * params_.wheel_radius;

  if (params_.open_loop)
  {
    odometry_.updateOpenLoop(linear_command_x, linear_command_y, angular_command, time);
  }
  else
  {
    const double front_left_feedback = registered_front_left_wheel_handle_->feedback.get().get_value();
    const double front_right_feedback = registered_front_right_wheel_handle_->feedback.get().get_value();
    const double rear_left_feedback = registered_rear_left_wheel_handle_->feedback.get().get_value();
    const double rear_right_feedback = registered_rear_right_wheel_handle_->feedback.get().get_value();

    if (std::isnan(front_left_feedback) || std::isnan(front_right_feedback) || std::isnan(rear_left_feedback) ||
        std::isnan(rear_right_feedback))
    {
      RCLCPP_ERROR(logger, "One of the wheel feedbacks %s is invalid", feedback_type());
      return controller_interface::return_type::ERROR;
    }

    if (params_.position_feedback)
    {
      odometry_.update(front_left_feedback, front_right_feedback, rear_left_feedback, rear_right_feedback, time);
    }
    else
    {
      odometry_.updateFromVelocity(front_left_feedback * wheel_radius * period.seconds(),
                                   front_right_feedback * wheel_radius * period.seconds(),
                                   rear_left_feedback * wheel_radius * period.seconds(),
                                   rear_right_feedback * wheel_radius * period.seconds(), time);
    }
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  bool should_publish = false;
  try
  {
    if (previous_publish_timestamp_ + publish_period_ < time)
    {
      previous_publish_timestamp_ += publish_period_;
      should_publish = true;
    }
  }
  catch (const std::runtime_error &)
  {
    // Handle exceptions when the time source changes and initialize publish timestamp
    previous_publish_timestamp_ = time;
    should_publish = true;
  }

  if (should_publish)
  {
    if (realtime_odometry_publisher_->trylock())
    {
      auto& odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = time;
      odometry_message.pose.pose.position.x = odometry_.getX();
      odometry_message.pose.pose.position.y = odometry_.getY();
      odometry_message.pose.pose.orientation.x = orientation.x();
      odometry_message.pose.pose.orientation.y = orientation.y();
      odometry_message.pose.pose.orientation.z = orientation.z();
      odometry_message.pose.pose.orientation.w = orientation.w();
      odometry_message.twist.twist.linear.x = odometry_.getLinearX();
      odometry_message.twist.twist.linear.y = odometry_.getLinearY();
      odometry_message.twist.twist.angular.z = odometry_.getAngular();
      realtime_odometry_publisher_->unlockAndPublish();
    }

    if (params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
    {
      auto& transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = time;
      transform.transform.translation.x = odometry_.getX();
      transform.transform.translation.y = odometry_.getY();
      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }
  }

  auto& last_command = previous_commands_.back().twist;
  auto& second_to_last_command = previous_commands_.front().twist;
  limiter_linear_x_.limit(linear_command_x, last_command.linear.x, second_to_last_command.linear.x, period.seconds());
  limiter_linear_y_.limit(linear_command_y, last_command.linear.y, second_to_last_command.linear.y, period.seconds());
  limiter_angular_.limit(angular_command, last_command.angular.z, second_to_last_command.angular.z, period.seconds());

  previous_commands_.pop();
  previous_commands_.emplace(command);

  //    Publish limited velocity
  if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
  {
    auto& limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
    limited_velocity_command.header.stamp = time;
    limited_velocity_command.twist = command.twist;
    realtime_limited_velocity_publisher_->unlockAndPublish();
  }

  // Compute wheels velocities:
  const double velocity_front_left =
      (linear_command_x - linear_command_y - (wheel_separation_x + wheel_separation_y) / 2. * angular_command) /
      wheel_radius;

  const double velocity_front_right =
      (linear_command_x + linear_command_y + (wheel_separation_x + wheel_separation_y) / 2. * angular_command) /
      wheel_radius;

  const double velocity_rear_left =
      (linear_command_x + linear_command_y - (wheel_separation_x + wheel_separation_y) / 2. * angular_command) /
      wheel_radius;

  const double velocity_rear_right =
      (linear_command_x - linear_command_y + (wheel_separation_x + wheel_separation_y) / 2. * angular_command) /
      wheel_radius;

  // Set wheels velocities:
  registered_front_left_wheel_handle_->velocity.get().set_value(velocity_front_left);
  registered_front_right_wheel_handle_->velocity.get().set_value(velocity_front_right);
  registered_rear_left_wheel_handle_->velocity.get().set_value(velocity_rear_left);
  registered_rear_right_wheel_handle_->velocity.get().set_value(velocity_rear_right);

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn MecanumDriveController::on_configure(const rclcpp_lifecycle::State&)
{
  auto logger = get_node()->get_logger();

  // update parameters if they have changed
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  std::string tf_prefix;
  if(params_.tf_frame_prefix_enable){
    if (params_.tf_frame_prefix != "")
    {
      tf_prefix = params_.tf_frame_prefix;
    }
    else
    {
      tf_prefix = std::string(get_node()->get_namespace());
    }

    if(tf_prefix == "/")
    {
      tf_prefix = "";
    }
    else
    {
      tf_prefix = tf_prefix + "/";
    }
  }

  front_left_wheel_name_ = params_.front_left_wheel_name;
  front_right_wheel_name_ = params_.front_right_wheel_name;
  rear_left_wheel_name_ = params_.rear_left_wheel_name;
  rear_right_wheel_name_ = params_.rear_right_wheel_name;

  if (front_left_wheel_name_.empty() || front_right_wheel_name_.empty() || rear_left_wheel_name_.empty() ||
      rear_right_wheel_name_.empty())
  {
    RCLCPP_ERROR(logger, "Wheel name parameter is empty!");
    return controller_interface::CallbackReturn::ERROR;
  }

  front_left_wheel_name_ = tf_prefix + front_left_wheel_name_;
  front_right_wheel_name_ = tf_prefix + front_right_wheel_name_;
  rear_left_wheel_name_ =  tf_prefix + rear_left_wheel_name_;
  rear_right_wheel_name_ = tf_prefix + rear_right_wheel_name_;
  params_.odom_frame_id = tf_prefix + params_.odom_frame_id;
  params_.base_frame_id = tf_prefix + params_.base_frame_id;

  const double wheel_separation_x = params_.wheel_separation_y_multiplier * params_.wheel_separation_x;
  const double wheel_separation_y = params_.wheel_separation_x_multiplier * params_.wheel_separation_y;
  const double wheel_radius = params_.wheel_radius_multiplier * params_.wheel_radius;

  odometry_.setWheelParams(wheel_separation_x, wheel_separation_y, wheel_radius);
  odometry_.setVelocityRollingWindowSize(params_.velocity_rolling_window_size);

  auto pose_diagonal = params_.pose_covariance_diagonal;
  std::copy(pose_diagonal.begin(), pose_diagonal.end(), params_.pose_covariance_diagonal.begin());

  auto twist_diagonal = params_.twist_covariance_diagonal;
  std::copy(twist_diagonal.begin(), twist_diagonal.end(), params_.twist_covariance_diagonal.begin());

  params_.open_loop = params_.open_loop;
  params_.position_feedback = params_.position_feedback;
  params_.enable_odom_tf = params_.enable_odom_tf;

  cmd_vel_timeout_ =
      std::chrono::milliseconds{ static_cast<int>(params_.cmd_vel_timeout * 1000.0) };
  publish_limited_velocity_ = params_.publish_limited_velocity;
  use_stamped_vel_ = params_.use_stamped_vel;

  try
  {
    limiter_linear_x_ = SpeedLimiter(params_.linear.x.has_velocity_limits,
                                     params_.linear.x.has_acceleration_limits,
                                     params_.linear.x.has_jerk_limits,
                                     params_.linear.x.min_velocity,
                                     params_.linear.x.max_velocity,
                                     params_.linear.x.min_acceleration,
                                     params_.linear.x.max_acceleration,
                                     params_.linear.x.min_jerk,
                                     params_.linear.x.max_jerk);
  }
  catch (const std::runtime_error& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring x linear speed limiter: %s", e.what());
  }

  try
  {
    limiter_linear_y_ = SpeedLimiter(params_.linear.y.has_velocity_limits,
                                     params_.linear.y.has_acceleration_limits,
                                     params_.linear.y.has_jerk_limits,
                                     params_.linear.y.min_velocity,
                                     params_.linear.y.max_velocity,
                                     params_.linear.y.min_acceleration,
                                     params_.linear.y.max_acceleration,
                                     params_.linear.y.min_jerk,
                                     params_.linear.y.max_jerk);
  }
  catch (const std::runtime_error& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring y linear speed limiter: %s", e.what());
  }

  try
  {
    limiter_angular_ = SpeedLimiter(params_.angular.z.has_velocity_limits,
                                    params_.angular.z.has_acceleration_limits,
                                    params_.angular.z.has_jerk_limits,
                                    params_.angular.z.min_velocity,
                                    params_.angular.z.max_velocity,
                                    params_.angular.z.min_acceleration,
                                    params_.angular.z.max_acceleration,
                                    params_.angular.z.min_jerk,
                                    params_.angular.z.max_jerk);
  }
  catch (const std::runtime_error& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring angular speed limiter: %s", e.what());
  }

  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (publish_limited_velocity_)
  {
    limited_velocity_publisher_ =
        get_node()->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }

  const Twist empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

  // initialize command subscriber
  if (use_stamped_vel_)
  {
    velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
        DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<Twist> msg) -> void {
          if (!subscriber_is_active_)
          {
            RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
          }
          if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
          {
            RCLCPP_WARN_ONCE(get_node()->get_logger(),
                             "Received TwistStamped with zero timestamp, setting it to current "
                             "time, this message will only be shown once");
            msg->header.stamp = get_node()->get_clock()->now();
          }
          received_velocity_msg_ptr_.set(std::move(msg));
        });
  }
  else
  {
    velocity_command_unstamped_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void {
          if (!subscriber_is_active_)
          {
            RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
          }

          // Write fake header in the stored stamped command
          std::shared_ptr<Twist> twist_stamped;
          received_velocity_msg_ptr_.get(twist_stamped);
          twist_stamped->twist = *msg;
          twist_stamped->header.stamp = get_node()->get_clock()->now();
        });
  }

  // initialize odometry publisher and messasge
  odometry_publisher_ =
      get_node()->create_publisher<nav_msgs::msg::Odometry>(DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odometry_publisher_);

  auto& odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = params_.odom_frame_id;
  odometry_message.child_frame_id = params_.base_frame_id;

  // limit the publication on the topics /odom and /tf
  publish_rate_ = params_.publish_rate;
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

  // initialize odom values zeros
  odometry_message.twist = geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ =
      get_node()->create_publisher<tf2_msgs::msg::TFMessage>(DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  auto& odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = params_.odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = params_.base_frame_id;

  previous_update_timestamp_ = get_node()->get_clock()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveController::on_activate(const rclcpp_lifecycle::State&)
{
  const auto front_left_result = configure_wheel(front_left_wheel_name_, registered_front_left_wheel_handle_);
  const auto front_right_result = configure_wheel(front_right_wheel_name_, registered_front_right_wheel_handle_);
  const auto rear_left_result = configure_wheel(rear_left_wheel_name_, registered_rear_left_wheel_handle_);
  const auto rear_right_result = configure_wheel(rear_right_wheel_name_, registered_rear_right_wheel_handle_);

  if (front_left_result == controller_interface::CallbackReturn::ERROR ||
      front_right_result == controller_interface::CallbackReturn::ERROR ||
      rear_left_result == controller_interface::CallbackReturn::ERROR ||
      rear_right_result == controller_interface::CallbackReturn::ERROR)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!(registered_front_left_wheel_handle_ && registered_front_right_wheel_handle_ &&
        registered_rear_left_wheel_handle_ && registered_rear_right_wheel_handle_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "One of wheel interfaces is non existent");
    return controller_interface::CallbackReturn::ERROR;
  }

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveController::on_deactivate(const rclcpp_lifecycle::State&)
{
  subscriber_is_active_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveController::on_cleanup(const rclcpp_lifecycle::State&)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveController::on_error(const rclcpp_lifecycle::State&)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool MecanumDriveController::reset()
{
  odometry_.resetOdometry();

  // release the old queue
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);

  registered_front_left_wheel_handle_.reset();
  registered_front_right_wheel_handle_.reset();
  registered_rear_left_wheel_handle_.reset();
  registered_rear_right_wheel_handle_.reset();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

controller_interface::CallbackReturn MecanumDriveController::on_shutdown(const rclcpp_lifecycle::State&)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

void MecanumDriveController::halt()
{
  registered_front_left_wheel_handle_->velocity.get().set_value(0.0);
  registered_front_right_wheel_handle_->velocity.get().set_value(0.0);
  registered_rear_left_wheel_handle_->velocity.get().set_value(0.0);
  registered_rear_right_wheel_handle_->velocity.get().set_value(0.0);
}

controller_interface::CallbackReturn
MecanumDriveController::configure_wheel(const std::string& wheel_name, std::unique_ptr<WheelHandle>& registered_handle)
{
  auto logger = get_node()->get_logger();

  if (wheel_name.empty())
  {
    RCLCPP_ERROR(logger, "No wheel name specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  // register handles
  const auto interface_name = feedback_type();
  const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_name, &interface_name](const auto& interface) {
        return interface.get_prefix_name() == wheel_name && interface.get_interface_name() == interface_name;
      });

  if (state_handle == state_interfaces_.cend())
  {
    RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto command_handle =
      std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&wheel_name](const auto& interface) {
        return interface.get_prefix_name() == wheel_name && interface.get_interface_name() == HW_IF_VELOCITY;
      });

  if (command_handle == command_interfaces_.end())
  {
    RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  registered_handle = std::make_unique<WheelHandle>(WheelHandle{ std::ref(*state_handle), std::ref(*command_handle) });

  return controller_interface::CallbackReturn::SUCCESS;
}
}  // namespace mecanum_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(mecanum_drive_controller::MecanumDriveController, controller_interface::ControllerInterface)
