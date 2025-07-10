cmake_minimum_required(VERSION 3.8)
project(demo_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

# Create executable
add_executable(ros2_cpp_node src/ros2_stm32_node.cpp)

# Link dependencies
target_include_directories(ros2_cpp_node PUBLIC ${rclcpp_INCLUDE_DIRS})
target_link_libraries(ros2_cpp_node ${rclcpp_LIBRARIES})
ament_target_dependencies(ros2_cpp_node 
  rclcpp 
  geometry_msgs 
  serial 
  tf2_ros 
  tf2 
  sensor_msgs 
  nav_msgs 
  std_msgs
)

# Install executable
install(TARGETS
  ros2_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()