.. _mecanum_drive_controller_userdoc:

.. Copied and adapted from diff_drive_controller (https://github.com/ros-controls/ros2_controllers)

mecanum_drive_controller
=====================

Controller for mobile robots with mecanum drive based on diff_drive_controller (https://github.com/ros-controls/ros2_controllers).
Input for control are robot body velocity commands which are translated to wheel commands for the mecanum drive base.
Odometry is computed from hardware feedback and published.

Velocity commands
-----------------

The controller works with a velocity twist from which it extracts the x and y components of the linear velocity and the z component of the angular velocity. Velocities on other components are ignored.

Hardware interface type
-----------------------

The controller works with wheel joints through a velocity interface.

Other features
--------------

    Realtime-safe implementation.
    Odometry publishing
    Task-space velocity, acceleration and jerk limits
    Automatic stop after command time-out
