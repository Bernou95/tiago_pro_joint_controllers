cmake_minimum_required(VERSION 3.8)
project(motion_controller_interface)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tiago_pro_joint_controllers_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(motion_controller_interface SHARED
  src/motion_controller_interface.cpp
)
target_compile_features(motion_controller_interface PUBLIC cxx_std_17)
target_include_directories(motion_controller_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motion_controller_interface
  controller_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  controller_manager_msgs
  std_msgs
  tiago_pro_joint_controllers_msgs
  realtime_tools
)

pluginlib_export_plugin_description_file(controller_interface motion_controller_interface.xml)

install(TARGETS motion_controller_interface
  EXPORT export_motion_controller_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_targets(export_motion_controller_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  controller_manager_msgs
  std_msgs
  tiago_pro_joint_controllers_msgs
  realtime_tools
)

ament_package()
