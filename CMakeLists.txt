cmake_minimum_required(VERSION 3.8)
project(openarm_impedance_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(openarm_hardware REQUIRED)
find_package(urdf REQUIRED)
find_package(pinocchio REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(openarm_impedance_controller SHARED
  src/robot_model.cpp
  src/goal_limits.cpp
  src/controller_params.cpp
  src/motor_gains.cpp
  src/openarm_impedance_controller.cpp
)

target_include_directories(openarm_impedance_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(openarm_impedance_controller
  rclcpp
  rclcpp_action
  controller_interface
  hardware_interface
  pluginlib
  control_msgs
  trajectory_msgs
  openarm_hardware
  urdf
  eigen3_cmake_module
  Eigen3
  realtime_tools
)

target_link_libraries(openarm_impedance_controller
  pinocchio::pinocchio
  yaml-cpp::yaml-cpp
)

pluginlib_export_plugin_description_file(controller_interface openarm_impedance_plugins.xml)

install(TARGETS openarm_impedance_controller
  EXPORT export_openarm_impedance
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES openarm_impedance_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()