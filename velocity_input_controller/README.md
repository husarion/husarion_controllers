
# Velocity Input Controller

The `VelocityInputController` is a ROS 2 chainable controller designed to process velocity commands from multiple sources and forward them to hardware interfaces or other chainable controllers.

## Publishers

- `{cmd_vel_topic}/source` [*std_msgs/msg/String*]: Publishes the source of the active velocity command.

## Subscribers

- `{cmd_vel_topic}` [geometry_msgs/msg/TwistStamped]: The base topic for command velocity, depends on `cmd_vel_topic` parameter.
- `{cmd_vel_topic}/{name}` [geometry_msgs/msg/TwistStamped]: For each input in `cmd_vel_inputs` parameter, controller will create subscriber to that command velocity.

## Parameters

- `cmd_vel_topic` [*string*, default: **cmd_vel**]: The base topic for command velocity. The controller will alway subscribe to this topic and assume it's source as 'unknown'.
- `cmd_vel_inputs` [*string_array*, default: **[]**]: List of command velocity inputs. The controller will subscribe to '{cmd_vel_topic}/{name}' for each name in the list. The order will indicate priority of the topics.
- `cmd_vel_timeout` [*double*, default: **0.5**]: Timeout for command velocity topics in seconds.
- `command_interface_linear` [*string*, default: **linear/velocity**]: Command interface for linear velocity.
- `command_interface_angular` [*string*, default: **angular/velocity**]: Command interface for angular velocity.

## Interfaces

### Exported Command Interfaces

- `{node_name}/linear/velocity`: Linear Velocity
- `{node_name}/angular/velocity`: Angular Velocity

## Example Usage

```yaml
velocity_input_controller:
  ros__parameters:
    cmd_vel_topic: "cmd_vel"
    cmd_vel_inputs: ["manual", "auto"]
    cmd_vel_timeout: 0.2
    command_interface_linear: "drive_controller/linear/velocity"
    command_interface_angular: "drive_controller/angular/velocity"
```
