cmake_minimum_required(VERSION 3.11)
project(twist_mux_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    controller_interface
    generate_parameter_library
    geometry_msgs
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

generate_parameter_library(twist_mux_controller_parameters
                           src/twist_mux_controller_parameters.yaml)

add_library(twist_mux_controller SHARED src/twist_mux_controller.cpp)

target_include_directories(
  twist_mux_controller
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/twist_mux_controller>)
target_link_libraries(twist_mux_controller
                      PUBLIC twist_mux_controller_parameters)
ament_target_dependencies(twist_mux_controller PUBLIC ${PACKAGE_DEPENDENCIES})
# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(twist_mux_controller
                           PRIVATE "TWIST_MUX_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface
                                         velocity_input_plugin.xml)

install(DIRECTORY include/ DESTINATION include/twist_mux_controller)

install(
  TARGETS ${PROJECT_NAME} twist_mux_controller_parameters
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
