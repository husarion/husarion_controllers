cmake_minimum_required(VERSION 3.5)
project(collision_monitor_controller)

# Dependencies ###

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(controller_interface REQUIRED)

# Header ###

nav2_package()

# Libraries and executables ###

include_directories(include)

set(dependencies
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    nav2_util
    nav2_costmap_2d
    nav2_msgs
    visualization_msgs
    controller_interface
    pluginlib
    rclcpp_lifecycle
    realtime_tools)

set(monitor_library_name collision_monitor_controller)

add_library(
  ${monitor_library_name} SHARED
  src/collision_monitor_node.cpp
  src/polygon.cpp
  src/velocity_polygon.cpp
  src/circle.cpp
  src/source.cpp
  src/scan.cpp
  src/pointcloud.cpp
  src/polygon_source.cpp
  src/range.cpp
  src/kinematics.cpp)

ament_target_dependencies(${monitor_library_name} ${dependencies})

# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(${monitor_library_name}
                           PRIVATE "COLLISION_MONITOR_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface
                                         collision_monitor_controller.xml)

# Install ###

install(
  TARGETS ${monitor_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

# Testing ###

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

# Ament stuff ###

ament_export_include_directories(include)
ament_export_libraries(${monitor_library_name})
ament_export_dependencies(${dependencies})

ament_package()
