
# Twist Mux Controller

The `TwistMuxController` is a ROS 2 chainable controller designed to process velocity commands from multiple sources and forward them to hardware interfaces or other chainable controllers.

## Publishers

- `{cmd_vel_topic}/source` [*std_msgs/msg/String*]: Publishes the source of the active velocity command. The source name corresponds to the key names in the `cmd_vel_inputs` parameter.

## Subscribers

- `{cmd_vel_topic}` [geometry_msgs/msg/TwistStamped]: The base topic for command velocity, depends on `cmd_vel_topic` parameter.
- `{cmd_vel_topic}/{name}` [geometry_msgs/msg/TwistStamped]: For each input in `cmd_vel_inputs` parameter, controller will create subscriber to that command velocity.

## Parameters

- `cmd_vel_inputs` [*dict*, default: **{}**]: List of command velocity inputs. Each key in the dictionary has a unique name and the following parameters:
  - `topic` [*string*, default: **{source_name}/cmd_vel**]: Name of the topic to subscribe to.
  - `timeout` [*double*, default: **0.5**]: Timeout for command velocity topic in seconds.
  - `priority` [*uint*, default: **0**]: The priority of the input in range [0, 255], where 0 is the lowest priority and 255 is the highest. Note, if there are multiple inputs with the same priority, the output will be chosen arbitrary.
- `command_interface_linear` [*string*, default: **linear/velocity**]: Command interface for linear velocity.
- `command_interface_angular` [*string*, default: **angular/velocity**]: Command interface for angular velocity.

## Interfaces

### Exported Command Interfaces

- `{node_name}/linear/velocity`: Linear Velocity
- `{node_name}/angular/velocity`: Angular Velocity

## Example Usage

```yaml
  twist_mux_controller:
    ros__parameters:
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
      command_interface_linear: "drive_controller/linear/velocity"
      command_interface_angular: "drive_controller/angular/velocity"
```
