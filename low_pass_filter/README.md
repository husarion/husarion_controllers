
# Low Pass Filter

The `LowPassFilter` is a ROS 2 chainable controller designed to apply a low-pass filter to state interfaces, smoothing noisy signals while maintaining real-time performance.

## Publishers

- `interface_values` [*control_msgs::msg::DynamicInterfaceValues*]: Publishes the filtered state interface values.

## Parameters

- `state_interface_prefix` [*string*, default: **""**]: Prefix to be prepended to the state interface names.
- `state_interface_names` [*string_array*, default: **[]**]: Names of state interfaces to be filtered.
- `sampling_frequency` [*double*, default: **100.0**]: Filter sampling frequency in Hz.
- `damping_frequency` [*double*, default: **0.1**]: Filter damping frequency in Hz (Is it?).
- `damping_intensity` [*double*, default: **0.707**]: Defines how sharp the filter is. 0.707 is the default value for a Butterworth filter.
- `publish_interface_values` [*bool*, default: **false**]: If true, the state interface values will be published to a topic.
- `zero_threshold` [*double*, default: **0.0025**]: If the filtered value is below this threshold, it will be set to zero.

## Interfaces

### Exported State Interfaces

- `{node_name}/{state_interface_prefix}/{state_interface_name}`: Filtered data.

## Example Usage

```yaml
low_pass_filter:
  ros__parameters:
    state_interface_prefix: <namespace>/imu
    state_interface_names: [angular_velocity.z]
    sampling_frequency: 100.0
    damping_frequency: 0.1
    damping_intensity: 15.0
    publish_interface_values: false
    zero_threshold: 0.0025
```
