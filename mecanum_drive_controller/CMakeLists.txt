# Copied and adapted from diff_drive_controller
# (https://github.com/ros-controls/ros2_controllers)

cmake_minimum_required(VERSION 3.11)
project(mecanum_drive_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    control_toolbox
    controller_interface
    generate_parameter_library
    geometry_msgs
    hardware_interface
    nav_msgs
    pluginlib
    rclcpp
    rclcpp_lifecycle
    rcpputils
    realtime_tools
    tf2
    tf2_msgs)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# get include dirs from control_toolbox for the custom validators
get_target_property(TB_INCLUDE_DIRS control_toolbox::rate_limiter_parameters
                    INTERFACE_INCLUDE_DIRECTORIES)
generate_parameter_library(
  mecanum_drive_controller_parameters
  src/mecanum_drive_controller_parameter.yaml
  ${TB_INCLUDE_DIRS}/control_toolbox/custom_validators.hpp)

add_library(mecanum_drive_controller SHARED src/mecanum_drive_controller.cpp
                                            src/odometry.cpp)
target_compile_features(mecanum_drive_controller PUBLIC cxx_std_17)
target_include_directories(
  mecanum_drive_controller
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/mecanum_drive_controller>)
target_link_libraries(
  mecanum_drive_controller PUBLIC mecanum_drive_controller_parameters
                                  control_toolbox::rate_limiter_parameters)
ament_target_dependencies(mecanum_drive_controller PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
pluginlib_export_plugin_description_file(controller_interface
                                         mecanum_drive_plugin.xml)

install(
  TARGETS mecanum_drive_controller mecanum_drive_controller_parameters
  EXPORT export_mecanum_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

ament_export_targets(export_mecanum_drive_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
