
# Twist Mux Controller

The `TwistMuxController` is a ROS 2 chainable controller designed to process velocity commands from multiple sources and forward them to hardware interfaces or other chainable controllers. The controllers support mobile platform controllers with holonomic and non-holonomic drive type.

## Publishers

- `~/source` [*std_msgs/msg/String*]: Publishes the source of the active velocity command. The source name corresponds to the key names in the `cmd_vel_inputs` parameter.

## Subscribers

- `{cmd_vel_inputs.source_key.topic}` [geometry_msgs/msg/TwistStamped]: For each input in `cmd_vel_inputs` parameter, controller will create subscriber to that command velocity. The topic name depends on the `topic` parameter. See [Example usage](#example-usage) for example topics.

## Parameters

- `holonomic` [*bool*, default: **false**]: Whether the robot drive type is holonomic or non-holonomic. This will affect the number of command interfaces.
- `cmd_vel_inputs` [*dict*, default: **{}**]: List of command velocity inputs. Each key in the dictionary has a unique name and the following parameters:
  - `topic` [*string*, default: **{source_name}/cmd_vel**]: Name of the topic to subscribe to.
  - `timeout` [*double*, default: **0.5**]: Timeout for command velocity topic in seconds.
  - `priority` [*uint*, default: **0**]: The priority of the input in range [0, 255], where 0 is the lowest priority and 255 is the highest. Note, if there are multiple inputs with the same priority, the output will be chosen arbitrary.
- `command_interface_linear_x` [*string*, default: **linear/x/velocity**]: Command interface for linear X velocity.
- `command_interface_linear_y` [*string*, default: **linear/y/velocity**]: Command interface for linear Y velocity. Available only if `holonomic` parameter is set to `true`.
- `command_interface_angular_z` [*string*, default: **angular/z/velocity**]: Command interface for angular velocity.

## Interfaces

### Exported Command Interfaces

- `{node_name}/linear/x/velocity`: Linear X Velocity
- `{node_name}/linear/y/velocity`: Linear Y Velocity. Available only if `holonomic` parameter is set to `true`.
- `{node_name}/angular/z/velocity`: Angular Velocity

## Example Usage

Controller configuration:

```yaml
  twist_mux_controller:
    ros__parameters:
      holonomic: false
      cmd_vel_inputs:
        manual:
          topic: manual/cmd_vel
          timeout: 0.2
          priority: 100
        autonomous:
          topic: autonomous/cmd_vel
          timeout: 0.2
          priority: 10
        unknown:
          topic: cmd_vel
          timeout: 0.2
          priority: 1
      command_interface_linear_x: drive_controller/linear/velocity
      command_interface_angular_z: drive_controller/angular/velocity
```

Above configuration will create subscribers for the following topics:

- `/manual/cmd_vel`
- `/autonomous/cmd_vel`
- `/cmd_vel`
