// Copyright (c) 2022 Samsung R&D Institute Russia
// Copyright 2026 Husarion sp. z o.o.
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

#include "nav2_collision_monitor/collision_monitor_node.hpp"

#include <exception>
#include <functional>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "tf2_ros/create_timer_ros.h"

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "nav2_collision_monitor/kinematics.hpp"

using namespace std::placeholders;

namespace nav2_collision_monitor
{

CollisionMonitor::CollisionMonitor()
: controller_interface::ChainableControllerInterface(),
  enabled_{true},
  process_active_(false),
  robot_action_prev_{DO_NOTHING, {-1.0, -1.0, -1.0}, ""},
  stop_stamp_{0, 0, RCL_ROS_TIME},
  stop_pub_timeout_(1.0, 0.0)
{
}

CollisionMonitor::~CollisionMonitor()
{
  polygons_.clear();
  sources_.clear();
}

controller_interface::InterfaceConfiguration CollisionMonitor::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(2);
  command_interfaces_config.names.push_back("drive_controller/linear/velocity");
  command_interfaces_config.names.push_back("drive_controller/angular/velocity");

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration CollisionMonitor::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type CollisionMonitor::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto cmd_vel_msg = *received_cmd_vel_msg_ptr_.readFromRT();

  if (cmd_vel_msg == nullptr) {
    std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), 0.0);
    return controller_interface::return_type::OK;
  }

  reference_interfaces_[0] = cmd_vel_msg->twist.linear.x;
  reference_interfaces_[1] = cmd_vel_msg->twist.angular.z;

  return controller_interface::return_type::OK;
}

controller_interface::return_type CollisionMonitor::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto result = std::vector<bool>();

  process({reference_interfaces_[0], 0.0, reference_interfaces_[1]}, time);

  result.push_back(command_interfaces_[0].set_value(reference_interfaces_[0]));
  result.push_back(command_interfaces_[1].set_value(reference_interfaces_[1]));

  if (!std::all_of(result.begin(), result.end(), [](bool success) { return success; })) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unable to set the command to one of the command handles!");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn CollisionMonitor::on_init()
{
  RCLCPP_INFO(get_node()->get_logger(), "Initializing");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CollisionMonitor::on_configure(
  const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_node()->get_logger(), "Configuring");
  reference_interfaces_.resize(2, std::numeric_limits<double>::quiet_NaN());

  // Transform buffer and listener initialization
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_node()->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node()->get_node_base_interface(), this->get_node()->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this->get_node(), true);

  std::string cmd_vel_in_topic;
  std::string cmd_vel_out_topic;
  std::string state_topic;

  // Obtaining ROS parameters
  if (!getParameters(cmd_vel_in_topic, cmd_vel_out_topic, state_topic)) {
    on_cleanup(state);
    return controller_interface::CallbackReturn::FAILURE;
  }

  cmd_vel_in_sub_ = std::make_unique<nav2_util::TwistSubscriber>(
    get_node()->shared_from_this(), cmd_vel_in_topic, 1,
    std::bind(&CollisionMonitor::cmdVelInCallbackUnstamped, this, std::placeholders::_1),
    std::bind(&CollisionMonitor::cmdVelInCallbackStamped, this, std::placeholders::_1));

  auto node = get_node()->shared_from_this();
  // cmd_vel_out_pub_ = std::make_unique<nav2_util::TwistPublisher>(node, cmd_vel_out_topic, 1);
  cmd_vel_out_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>(
    cmd_vel_out_topic, rclcpp::SystemDefaultsQoS());
  realtime_cmd_vel_out_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(
      cmd_vel_out_pub_);

  if (!state_topic.empty()) {
    state_pub_ = this->get_node()->create_publisher<nav2_msgs::msg::CollisionMonitorState>(
      state_topic, 1);
    realtime_state_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<nav2_msgs::msg::CollisionMonitorState>>(
        state_pub_);
  }

  collision_points_marker_pub_ =
    this->get_node()->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/collision_points_marker", 1);
  realtime_collision_points_marker_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<visualization_msgs::msg::MarkerArray>>(
      collision_points_marker_pub_);

  // Toggle service initialization
  toggle_cm_service_ = get_node()->create_service<nav2_msgs::srv::Toggle>(
    "~/toggle", std::bind(&CollisionMonitor::toggleCMServiceCallback, this, _1, _2, _3));

  nav2_util::declare_parameter_if_not_declared(
    node, "use_realtime_priority", rclcpp::ParameterValue(false));
  bool use_realtime_priority = false;
  node->get_parameter("use_realtime_priority", use_realtime_priority);
  if (use_realtime_priority) {
    try {
      nav2_util::setSoftRealTimePriority();
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
      on_cleanup(state);
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CollisionMonitor::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Activating");

  // Activating lifecycle publisher
  cmd_vel_out_pub_->on_activate();
  if (state_pub_) {
    state_pub_->on_activate();
  }
  collision_points_marker_pub_->on_activate();

  // Activating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->activate();
  }

  // Since polygons are being published when cmd_vel_in appears,
  // we need to publish polygons first time to display them at startup
  publishPolygons();

  // Activating main worker
  process_active_ = true;

  // Creating bond connection
  // createBond();

  RCLCPP_INFO(get_node()->get_logger(), "Activating done");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CollisionMonitor::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating");

  // Deactivating main worker
  process_active_ = false;

  // Reset action type to default after worker deactivating
  robot_action_prev_ = {DO_NOTHING, {-1.0, -1.0, -1.0}, ""};

  // Deactivating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->deactivate();
  }

  // Deactivating lifecycle publishers
  cmd_vel_out_pub_->on_deactivate();
  if (state_pub_) {
    state_pub_->on_deactivate();
  }
  collision_points_marker_pub_->on_deactivate();

  // Destroying bond connection
  // destroyBond();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CollisionMonitor::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Cleaning up");

  cmd_vel_in_sub_.reset();
  cmd_vel_out_pub_.reset();
  state_pub_.reset();
  collision_points_marker_pub_.reset();

  polygons_.clear();
  sources_.clear();

  tf_listener_.reset();
  tf_buffer_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CollisionMonitor::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Shutting down");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CollisionMonitor::on_error(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Error occurred");

  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> CollisionMonitor::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name() + std::string("/linear/x"), hardware_interface::HW_IF_VELOCITY,
    &reference_interfaces_[0]));
  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name() + std::string("/angular/z"), hardware_interface::HW_IF_VELOCITY,
    &reference_interfaces_[1]));

  return reference_interfaces;
}

void CollisionMonitor::cmdVelInCallbackStamped(geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // If message contains NaN or Inf, ignore
  if (!nav2_util::validateTwist(*msg)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Velocity message contains NaNs or Infs! Ignoring as invalid!");
    return;
  }

  received_cmd_vel_msg_ptr_.writeFromNonRT(msg);
}

void CollisionMonitor::cmdVelInCallbackUnstamped(geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto twist_stamped = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_stamped->twist = *msg;
  cmdVelInCallbackStamped(twist_stamped);
}

void CollisionMonitor::publishVelocity(const Action & robot_action, const rclcpp::Time & curr_time)
{
  if (robot_action.req_vel.isZero()) {
    if (!robot_action_prev_.req_vel.isZero()) {
      // Robot just stopped: saving stop timestamp and continue
      stop_stamp_ = this->get_node()->now();
    } else if (this->get_node()->now() - stop_stamp_ > stop_pub_timeout_) {
      // More than stop_pub_timeout_ passed after robot has been stopped.
      // Cease publishing output cmd_vel and set references to zero.

      // Update references
      reference_interfaces_[0] = 0.0;
      reference_interfaces_[1] = 0.0;
      return;
    }
  }

  auto cmd_vel_out_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel_out_msg->header.stamp = curr_time;
  cmd_vel_out_msg->header.frame_id = this->get_node()->get_parameter("base_frame_id").as_string();
  cmd_vel_out_msg->twist.linear.x = robot_action.req_vel.x;
  cmd_vel_out_msg->twist.linear.y = robot_action.req_vel.y;
  cmd_vel_out_msg->twist.angular.z = robot_action.req_vel.tw;
  // linear.z, angular.x and angular.y will remain 0.0

  // cmd_vel_out_pub_->publish(std::move(cmd_vel_out_msg));
  realtime_cmd_vel_out_pub_->msg_ = *cmd_vel_out_msg;
  realtime_cmd_vel_out_pub_->unlockAndPublish();

  // Update references
  reference_interfaces_[0] = robot_action.req_vel.x;
  reference_interfaces_[1] = robot_action.req_vel.tw;
}

bool CollisionMonitor::getParameters(
  std::string & cmd_vel_in_topic, std::string & cmd_vel_out_topic, std::string & state_topic)
{
  std::string base_frame_id, odom_frame_id;
  tf2::Duration transform_tolerance;
  rclcpp::Duration source_timeout(2.0, 0.0);

  auto node = get_node()->shared_from_this();

  nav2_util::declare_parameter_if_not_declared(
    node, "cmd_vel_in_topic", rclcpp::ParameterValue("cmd_vel_smoothed"));
  cmd_vel_in_topic = get_node()->get_parameter("cmd_vel_in_topic").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, "cmd_vel_out_topic", rclcpp::ParameterValue("cmd_vel"));
  cmd_vel_out_topic = get_node()->get_parameter("cmd_vel_out_topic").as_string();
  nav2_util::declare_parameter_if_not_declared(node, "state_topic", rclcpp::ParameterValue(""));
  state_topic = get_node()->get_parameter("state_topic").as_string();

  nav2_util::declare_parameter_if_not_declared(
    node, "base_frame_id", rclcpp::ParameterValue("base_footprint"));
  base_frame_id = get_node()->get_parameter("base_frame_id").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, "odom_frame_id", rclcpp::ParameterValue("odom"));
  odom_frame_id = get_node()->get_parameter("odom_frame_id").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, "transform_tolerance", rclcpp::ParameterValue(0.1));
  transform_tolerance =
    tf2::durationFromSec(get_node()->get_parameter("transform_tolerance").as_double());
  nav2_util::declare_parameter_if_not_declared(node, "source_timeout", rclcpp::ParameterValue(2.0));
  source_timeout =
    rclcpp::Duration::from_seconds(get_node()->get_parameter("source_timeout").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, "base_shift_correction", rclcpp::ParameterValue(true));
  const bool base_shift_correction = get_node()->get_parameter("base_shift_correction").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, "stop_pub_timeout", rclcpp::ParameterValue(1.0));
  stop_pub_timeout_ =
    rclcpp::Duration::from_seconds(get_node()->get_parameter("stop_pub_timeout").as_double());

  if (!configureSources(
        base_frame_id, odom_frame_id, transform_tolerance, source_timeout, base_shift_correction)) {
    return false;
  }

  if (!configurePolygons(base_frame_id, transform_tolerance)) {
    return false;
  }

  return true;
}

bool CollisionMonitor::configurePolygons(
  const std::string & base_frame_id, const tf2::Duration & transform_tolerance)
{
  try {
    auto node = get_node()->shared_from_this();

    // Leave it to be not initialized: to intentionally cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(node, "polygons", rclcpp::PARAMETER_STRING_ARRAY);
    std::vector<std::string> polygon_names =
      get_node()->get_parameter("polygons").as_string_array();
    for (std::string polygon_name : polygon_names) {
      // Leave it not initialized: the will cause an error if it will not set
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name + ".type", rclcpp::PARAMETER_STRING);
      const std::string polygon_type =
        get_node()->get_parameter(polygon_name + ".type").as_string();

      if (polygon_type == "polygon") {
        polygons_.push_back(std::make_shared<Polygon>(
          node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      } else if (polygon_type == "circle") {
        polygons_.push_back(std::make_shared<Circle>(
          node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      } else if (polygon_type == "velocity_polygon") {
        polygons_.push_back(std::make_shared<VelocityPolygon>(
          node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      } else {  // Error if something else
        RCLCPP_ERROR(
          get_node()->get_logger(), "[%s]: Unknown polygon type: %s", polygon_name.c_str(),
          polygon_type.c_str());
        return false;
      }

      // Configure last added polygon
      if (!polygons_.back()->configure()) {
        return false;
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

bool CollisionMonitor::configureSources(
  const std::string & base_frame_id, const std::string & odom_frame_id,
  const tf2::Duration & transform_tolerance, const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
{
  try {
    auto node = get_node()->shared_from_this();

    // Leave it to be not initialized: to intentionally cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(
      node, "observation_sources", rclcpp::PARAMETER_STRING_ARRAY);
    std::vector<std::string> source_names =
      get_node()->get_parameter("observation_sources").as_string_array();
    for (std::string source_name : source_names) {
      nav2_util::declare_parameter_if_not_declared(
        node, source_name + ".type",
        rclcpp::ParameterValue("scan"));  // Laser scanner by default
      const std::string source_type = get_node()->get_parameter(source_name + ".type").as_string();

      if (source_type == "scan") {
        std::shared_ptr<Scan> s = std::make_shared<Scan>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id, transform_tolerance,
          source_timeout, base_shift_correction);

        s->configure();

        sources_.push_back(s);
      } else if (source_type == "pointcloud") {
        std::shared_ptr<PointCloud> p = std::make_shared<PointCloud>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id, transform_tolerance,
          source_timeout, base_shift_correction);

        p->configure();

        sources_.push_back(p);
      } else if (source_type == "range") {
        std::shared_ptr<Range> r = std::make_shared<Range>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id, transform_tolerance,
          source_timeout, base_shift_correction);

        r->configure();

        sources_.push_back(r);
      } else if (source_type == "polygon") {
        std::shared_ptr<PolygonSource> ps = std::make_shared<PolygonSource>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id, transform_tolerance,
          source_timeout, base_shift_correction);
        ps->configure();

        sources_.push_back(ps);
      } else {  // Error if something else
        RCLCPP_ERROR(
          get_node()->get_logger(), "[%s]: Unknown source type: %s", source_name.c_str(),
          source_type.c_str());
        return false;
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

void CollisionMonitor::process(const Velocity & cmd_vel_in, const rclcpp::Time & curr_time)
{
  // Do nothing if main worker in non-active state
  if (!process_active_) {
    return;
  }

  // Points array collected from different data sources in a robot base frame
  std::unordered_map<std::string, std::vector<Point>> sources_collision_points_map;

  // By default - there is no action
  Action robot_action{DO_NOTHING, cmd_vel_in, ""};
  // Polygon causing robot action (if any)
  std::shared_ptr<Polygon> action_polygon;

  // Fill collision points array from different data sources
  auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
  for (std::shared_ptr<Source> source : sources_) {
    auto iter =
      sources_collision_points_map.insert({source->getSourceName(), std::vector<Point>()});

    if (source->getEnabled()) {
      if (
        !source->getData(curr_time, iter.first->second) &&
        source->getSourceTimeout().seconds() != 0.0) {
        action_polygon = nullptr;
        robot_action.polygon_name = "invalid source";
        robot_action.action_type = STOP;
        robot_action.req_vel.x = 0.0;
        robot_action.req_vel.y = 0.0;
        robot_action.req_vel.tw = 0.0;
        break;
      }
    }

    if (collision_points_marker_pub_->get_subscription_count() > 0) {
      // visualize collision points with markers
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = get_node()->get_parameter("base_frame_id").as_string();
      marker.header.stamp = rclcpp::Time(0, 0);
      marker.ns = "collision_points_" + source->getSourceName();
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.02;
      marker.scale.y = 0.02;
      marker.color.r = 1.0;
      marker.color.a = 1.0;
      marker.lifetime = rclcpp::Duration(0, 0);
      marker.frame_locked = true;

      for (const auto & point : iter.first->second) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;
        marker.points.push_back(p);
      }
      marker_array->markers.push_back(marker);
    }
  }

  if (collision_points_marker_pub_->get_subscription_count() > 0) {
    // collision_points_marker_pub_->publish(std::move(marker_array));
    realtime_collision_points_marker_pub_->msg_ = *marker_array;
    realtime_collision_points_marker_pub_->unlockAndPublish();
  }

  for (std::shared_ptr<Polygon> polygon : polygons_) {
    if (!polygon->getEnabled() || !enabled_) {
      continue;
    }
    if (robot_action.action_type == STOP) {
      // If robot already should stop, do nothing
      break;
    }

    // Update polygon coordinates
    polygon->updatePolygon(cmd_vel_in);

    const ActionType at = polygon->getActionType();
    if (at == STOP || at == SLOWDOWN || at == LIMIT) {
      // Process STOP/SLOWDOWN for the selected polygon
      if (processStopSlowdownLimit(
            polygon, sources_collision_points_map, cmd_vel_in, robot_action)) {
        action_polygon = polygon;
      }
    } else if (at == APPROACH) {
      // Process APPROACH for the selected polygon
      if (processApproach(polygon, sources_collision_points_map, cmd_vel_in, robot_action)) {
        action_polygon = polygon;
      }
    }
  }

  if ((robot_action.polygon_name != robot_action_prev_.polygon_name) && enabled_) {
    // Report changed robot behavior
    notifyActionState(robot_action, action_polygon);
  }

  // Publish required robot velocity
  publishVelocity(robot_action, curr_time);

  // Publish polygons for better visualization
  publishPolygons();

  robot_action_prev_ = robot_action;
}

bool CollisionMonitor::processStopSlowdownLimit(
  const std::shared_ptr<Polygon> polygon,
  const std::unordered_map<std::string, std::vector<Point>> & sources_collision_points_map,
  const Velocity & velocity, Action & robot_action) const
{
  if (!polygon->isShapeSet()) {
    return false;
  }

  if (polygon->getPointsInside(sources_collision_points_map) >= polygon->getMinPoints()) {
    if (polygon->getActionType() == STOP) {
      // Setting up zero velocity for STOP model
      robot_action.polygon_name = polygon->getName();
      robot_action.action_type = STOP;
      robot_action.req_vel.x = 0.0;
      robot_action.req_vel.y = 0.0;
      robot_action.req_vel.tw = 0.0;
      return true;
    } else if (polygon->getActionType() == SLOWDOWN) {
      const Velocity safe_vel = velocity * polygon->getSlowdownRatio();
      // Check that currently calculated velocity is safer than
      // chosen for previous shapes one
      if (safe_vel < robot_action.req_vel) {
        robot_action.polygon_name = polygon->getName();
        robot_action.action_type = SLOWDOWN;
        robot_action.req_vel = safe_vel;
        return true;
      }
    } else {  // Limit
      // Compute linear velocity
      const double linear_vel = std::hypot(velocity.x, velocity.y);  // absolute
      Velocity safe_vel;
      double ratio = 1.0;

      // Calculate the most restrictive ratio to preserve curvature
      if (linear_vel != 0.0) {
        ratio = std::min(ratio, polygon->getLinearLimit() / linear_vel);
      }
      if (velocity.tw != 0.0) {
        ratio = std::min(ratio, polygon->getAngularLimit() / std::abs(velocity.tw));
      }
      ratio = std::clamp(ratio, 0.0, 1.0);
      // Apply the same ratio to all components to preserve curvature
      safe_vel = velocity * ratio;
      // Check that currently calculated velocity is safer than
      // chosen for previous shapes one
      if (safe_vel < robot_action.req_vel) {
        robot_action.polygon_name = polygon->getName();
        robot_action.action_type = LIMIT;
        robot_action.req_vel = safe_vel;
        return true;
      }
    }
  }

  return false;
}

bool CollisionMonitor::processApproach(
  const std::shared_ptr<Polygon> polygon,
  const std::unordered_map<std::string, std::vector<Point>> & sources_collision_points_map,
  const Velocity & velocity, Action & robot_action) const
{
  if (!polygon->isShapeSet()) {
    return false;
  }

  // Obtain time before a collision
  const double collision_time = polygon->getCollisionTime(sources_collision_points_map, velocity);
  if (collision_time >= 0.0) {
    // If collision will occur, reduce robot speed
    const double change_ratio = collision_time / polygon->getTimeBeforeCollision();
    const Velocity safe_vel = velocity * change_ratio;
    // Check that currently calculated velocity is safer than
    // chosen for previous shapes one
    if (safe_vel < robot_action.req_vel) {
      robot_action.polygon_name = polygon->getName();
      robot_action.action_type = APPROACH;
      robot_action.req_vel = safe_vel;
      return true;
    }
  }

  return false;
}

void CollisionMonitor::notifyActionState(
  const Action & robot_action, const std::shared_ptr<Polygon> action_polygon) const
{
  if (robot_action.action_type == STOP) {
    if (robot_action.polygon_name == "invalid source") {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Robot to stop due to invalid source."
        " Either due to data not published yet, or to lack of new data received within the"
        " sensor timeout, or if impossible to transform data to base frame");
    } else {
      RCLCPP_INFO(
        get_node()->get_logger(), "Robot to stop due to %s polygon",
        action_polygon->getName().c_str());
    }
  } else if (robot_action.action_type == SLOWDOWN) {
    RCLCPP_INFO(
      get_node()->get_logger(), "Robot to slowdown for %f percents due to %s polygon",
      action_polygon->getSlowdownRatio() * 100, action_polygon->getName().c_str());
  } else if (robot_action.action_type == LIMIT) {
    RCLCPP_INFO(
      get_node()->get_logger(), "Robot to limit speed due to %s polygon",
      action_polygon->getName().c_str());
  } else if (robot_action.action_type == APPROACH) {
    RCLCPP_INFO(
      get_node()->get_logger(), "Robot to approach for %f seconds away from collision",
      action_polygon->getTimeBeforeCollision());
  } else {  // robot_action.action_type == DO_NOTHING
    RCLCPP_INFO(get_node()->get_logger(), "Robot to continue normal operation");
  }

  if (state_pub_) {
    std::unique_ptr<nav2_msgs::msg::CollisionMonitorState> state_msg =
      std::make_unique<nav2_msgs::msg::CollisionMonitorState>();
    state_msg->polygon_name = robot_action.polygon_name;
    state_msg->action_type = robot_action.action_type;

    realtime_state_pub_->msg_ = *state_msg;
    realtime_state_pub_->unlockAndPublish();
  }
}

void CollisionMonitor::publishPolygons() const
{
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    if (polygon->getEnabled() || !enabled_) {
      polygon->publish();
    }
  }
}

void CollisionMonitor::toggleCMServiceCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::Toggle::Request> request,
  std::shared_ptr<nav2_msgs::srv::Toggle::Response> response)
{
  enabled_ = request->enable;

  std::stringstream message;
  message << "Collision monitor toggled " << (enabled_ ? "on" : "off") << " successfully";

  response->success = true;
  response->message = message.str();
}

}  // namespace nav2_collision_monitor

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  nav2_collision_monitor::CollisionMonitor, controller_interface::ChainableControllerInterface)
