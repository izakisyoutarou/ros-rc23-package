cmake_minimum_required(VERSION 3.5)
project(injection_param_calculator)

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
find_package(std_msgs REQUIRED)
find_package(socketcan_interface_msg REQUIRED)
find_package(injection_interface_msg REQUIRED)
find_package(utilities REQUIRED)

add_library(injection_param_calculator_node 
  src/injection_param_calculator.cpp
)
target_include_directories(injection_param_calculator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(injection_param_calculator_node
  rclcpp
  std_msgs
  socketcan_interface_msg
  injection_interface_msg
  utilities
)

target_compile_definitions(injection_param_calculator_node PRIVATE "INJECTION_PARAM_CALCULATOR_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS injection_param_calculator_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

ament_export_include_directories(
  include
)
ament_export_libraries(
  injection_param_calculator_node
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(
    rclcpp
    std_msgs
    socketcan_interface_msg
    injection_interface_msg
    utilities
)
ament_package()
