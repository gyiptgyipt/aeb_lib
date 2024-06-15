cmake_minimum_required(VERSION 3.8)
project(aeb)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)


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


add_executable(aeb  src/rom_aeb.cpp src/main.cpp)
ament_target_dependencies(aeb rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs)


add_executable(aeb_lib src/aeb_lib.cpp)
ament_target_dependencies(aeb_lib rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs)

# add_executable(laser  src/laser_angle_filter.cpp src/main.cpp)
# ament_target_dependencies(laser rclcpp std_msgs sensor_msgs nav_msgs )

install(TARGETS
  aeb
  aeb_lib
  #laser
  DESTINATION lib/${PROJECT_NAME})




ament_package()
