// Copyright (c) 2022 Samsung R&D Institute Russia
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

#ifndef COLLISION_MONITOR_CONTROLLER__COLLISION_MONITOR_NODE_HPP_
#define COLLISION_MONITOR_CONTROLLER__COLLISION_MONITOR_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_msgs/msg/collision_monitor_state.hpp"
#include "nav2_msgs/srv/toggle.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/twist_publisher.hpp"
#include "nav2_util/twist_subscriber.hpp"

#include "collision_monitor_controller/circle.hpp"
#include "collision_monitor_controller/pointcloud.hpp"
#include "collision_monitor_controller/polygon.hpp"
#include "collision_monitor_controller/polygon_source.hpp"
#include "collision_monitor_controller/range.hpp"
#include "collision_monitor_controller/scan.hpp"
#include "collision_monitor_controller/source.hpp"
#include "collision_monitor_controller/types.hpp"
#include "collision_monitor_controller/velocity_polygon.hpp"

namespace collision_monitor_controller
{

/**
 * @brief Collision Monitor ROS2 node
 */
class CollisionMonitor : public controller_interface::ChainableControllerInterface
{
public:
  /**
   * @brief Constructor for the collision_monitor_controller::CollisionMonitor
   * @param options Additional options to control creation of the node.
   */
  CollisionMonitor();
  /**
   * @brief Destructor for the collision_monitor_controller::CollisionMonitor
   */
  ~CollisionMonitor();

protected:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Chainable controller replaces update() with the following two functions
  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief: Initializes ROS-parameters
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  controller_interface::CallbackReturn on_init() override;
  /**
   * @brief: Initializes and obtains ROS-parameters, creates main subscribers and publishers,
   * creates polygons and data sources objects
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Activates LifecyclePublishers, polygons and main processor, creates bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Deactivates LifecyclePublishers, polygons and main processor, destroys bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Resets all subscribers/publishers, polygons/data sources arrays
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called in shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called in error state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  /**
   * @brief Callback for input cmd_vel
   * @param msg Input cmd_vel message
   */
  void cmdVelInCallbackStamped(geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void cmdVelInCallbackUnstamped(geometry_msgs::msg::Twist::SharedPtr msg);
  /**
   * @brief Publishes output cmd_vel. If robot was stopped more than stop_pub_timeout_ seconds,
   * quit to publish 0-velocity.
   * @param robot_action Robot action to publish
   * @param header TwistStamped header to use
   */
  void publishVelocity(const Action & robot_action, const rclcpp::Time & curr_time);

  /**
   * @brief Supporting routine obtaining all ROS-parameters
   * @param cmd_vel_in_topic Output name of cmd_vel_in topic
   * @param cmd_vel_out_topic Output name of cmd_vel_out topic
   * is required.
   * @param state_topic topic name for publishing collision monitor state
   * @return True if all parameters were obtained or false in failure case
   */
  bool getParameters(
    std::string & cmd_vel_in_topic, std::string & cmd_vel_out_topic, std::string & state_topic);
  /**
   * @brief Supporting routine creating and configuring all polygons
   * @param base_frame_id Robot base frame ID
   * @param transform_tolerance Transform tolerance
   * @return True if all polygons were configured successfully or false in failure case
   */
  bool configurePolygons(
    const std::string & base_frame_id, const tf2::Duration & transform_tolerance);
  /**
   * @brief Supporting routine creating and configuring all data sources
   * @param base_frame_id Robot base frame ID
   * @param odom_frame_id Odometry frame ID. Used as global frame to get
   * source->base time interpolated transform.
   * @param transform_tolerance Transform tolerance
   * @param source_timeout Maximum time interval in which data is considered valid
   * @param base_shift_correction Whether to correct source data towards to base frame movement,
   * considering the difference between current time and latest source time
   * @return True if all sources were configured successfully or false in failure case
   */
  bool configureSources(
    const std::string & base_frame_id, const std::string & odom_frame_id,
    const tf2::Duration & transform_tolerance, const rclcpp::Duration & source_timeout,
    const bool base_shift_correction);

  /**
   * @brief Main processing routine
   * @param cmd_vel_in Input desired robot velocity
   * @param header Twist header
   */
  void process(const Velocity & cmd_vel_in, const rclcpp::Time & curr_time);

  /**
   * @brief Processes the polygon of STOP, SLOWDOWN and LIMIT action type
   * @param polygon Polygon to process
   * @param sources_collision_points_map Map containing source name as key and
   * array of source's 2D obstacle points as value
   * @param velocity Desired robot velocity
   * @param robot_action Output processed robot action
   * @return True if returned action is caused by current polygon, otherwise false
   */
  bool processStopSlowdownLimit(
    const std::shared_ptr<Polygon> polygon,
    const std::unordered_map<std::string, std::vector<Point>> & sources_collision_points_map,
    const Velocity & velocity, Action & robot_action) const;

  /**
   * @brief Processes APPROACH action type
   * @param polygon Polygon to process
   * @param sources_collision_points_map Map containing source name as key and
   * array of source's 2D obstacle points as value
   * @param velocity Desired robot velocity
   * @param robot_action Output processed robot action
   * @return True if returned action is caused by current polygon, otherwise false
   */
  bool processApproach(
    const std::shared_ptr<Polygon> polygon,
    const std::unordered_map<std::string, std::vector<Point>> & sources_collision_points_map,
    const Velocity & velocity, Action & robot_action) const;

  /**
   * @brief Log and publish current robot action and polygon
   * @param robot_action Robot action to notify
   * @param action_polygon Pointer to a polygon causing a selected action
   */
  void notifyActionState(
    const Action & robot_action, const std::shared_ptr<Polygon> action_polygon) const;

  /**
   * @brief Polygons publishing routine. Made for visualization.
   */
  void publishPolygons() const;

  /**
   * @brief Enable/disable collision monitor service callback
   * @param request Service request
   * @param response Service response
   */
  void toggleCMServiceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::Toggle::Request> request,
    std::shared_ptr<nav2_msgs::srv::Toggle::Response> response);

  // ----- Variables -----

  /// @brief TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  /// @brief TF listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// @brief Polygons array
  std::vector<std::shared_ptr<Polygon>> polygons_;

  /// @brief Data sources array
  std::vector<std::shared_ptr<Source>> sources_;

  // Input/output speed controls
  /// @brief Input cmd_vel subscriber
  std::unique_ptr<nav2_util::TwistSubscriber> cmd_vel_in_sub_;
  /// @brief Output cmd_vel publisher
  // std::unique_ptr<nav2_util::TwistPublisher> cmd_vel_out_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    cmd_vel_out_pub_;
  realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    realtime_cmd_vel_out_pub_;

  /// @brief CollisionMonitor state publisher
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::CollisionMonitorState>::SharedPtr state_pub_;
  realtime_tools::RealtimePublisher<nav2_msgs::msg::CollisionMonitorState>::SharedPtr
    realtime_state_pub_;

  /// @brief Collision points marker publisher
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    collision_points_marker_pub_;
  realtime_tools::RealtimePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    realtime_collision_points_marker_pub_;

  /// @brief Enable/disable collision monitor service
  rclcpp::Service<nav2_msgs::srv::Toggle>::SharedPtr toggle_cm_service_;

  /// @brief Whether collision monitor is enabled
  bool enabled_;

  /// @brief Whether main routine is active
  bool process_active_;

  /// @brief Previous robot action
  Action robot_action_prev_;
  /// @brief Latest timestamp when robot has 0-velocity
  rclcpp::Time stop_stamp_;
  /// @brief Timeout after which 0-velocity ceases to be published
  rclcpp::Duration stop_pub_timeout_;

  realtime_tools::RealtimeBuffer<geometry_msgs::msg::TwistStamped::SharedPtr>
    received_cmd_vel_msg_ptr_{nullptr};
};  // class CollisionMonitor

}  // namespace collision_monitor_controller

#endif  // COLLISION_MONITOR_CONTROLLER__COLLISION_MONITOR_NODE_HPP_
