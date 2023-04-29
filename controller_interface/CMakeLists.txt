cmake_minimum_required(VERSION 3.5)
project(controller_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(utilities REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(socketcan_interface_msg REQUIRED)
find_package(controller_interface_msg REQUIRED)

add_library(controller_interface_node
  src/controller_interface_node.cpp
  src/udp.cpp
  src/send_udp.cpp
  src/super_command.cpp
)

target_compile_definitions(controller_interface_node
  PRIVATE "CONTROLLER_INTERFACE_LIBRARY"
)

target_compile_options(controller_interface_node PUBLIC -Wall)

ament_target_dependencies(controller_interface_node
  rclcpp
  utilities
  std_msgs
  sensor_msgs
  geometry_msgs
  socketcan_interface_msg
  controller_interface_msg
)

target_include_directories(controller_interface_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(NOT WIN32)
  ament_environment_hooks(
     "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS controller_interface_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  utilities
  std_msgs
  sensor_msgs
  geometry_msgs
  socketcan_interface_msg
  controller_interface_msg
)

ament_package()

