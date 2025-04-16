cmake_minimum_required(VERSION 3.8)
project(low_pass_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    control_toolbox
    controller_interface
    generate_parameter_library
    hardware_interface
    pluginlib
    realtime_tools
    rclcpp
    rclcpp_lifecycle)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

generate_parameter_library(low_pass_filter_parameters
                           src/low_pass_filter_parameters.yaml)

add_library(low_pass_filter SHARED src/low_pass_filter.cpp)

target_include_directories(
  low_pass_filter PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/low_pass_filter>)
target_link_libraries(low_pass_filter PUBLIC low_pass_filter_parameters)
ament_target_dependencies(low_pass_filter PUBLIC ${PACKAGE_DEPENDENCIES})
# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(low_pass_filter
                           PRIVATE "low_pass_filter_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface
                                         low_pass_filter_plugin.xml)

install(DIRECTORY include/ DESTINATION include/low_pass_filter)

install(
  TARGETS ${PROJECT_NAME} low_pass_filter_parameters
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# if(BUILD_TESTING) find_package(ament_lint_auto REQUIRED)
# set(ament_cmake_copyright_FOUND TRUE) set(ament_cmake_cpplint_FOUND TRUE)
# ament_lint_auto_find_test_dependencies() endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
