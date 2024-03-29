cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(robot_patrol REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
  "action/GoToPose.action"
  DEPENDENCIES sensor_msgs geometry_msgs
)

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

# Excecutables
add_executable(patrol src/patrol.cpp)
add_executable(direction_service src/direction_service.cpp)
add_executable(test_service src/test_service.cpp)
add_executable(patrol_with_service src/patrol_with_service.cpp)
add_executable(go_to_pose_action src/go_to_pose_action.cpp)

# Target dependencies
ament_target_dependencies(patrol rclcpp sensor_msgs geometry_msgs)
ament_target_dependencies(direction_service rclcpp robot_patrol)
ament_target_dependencies(test_service rclcpp robot_patrol sensor_msgs)
ament_target_dependencies(patrol_with_service rclcpp robot_patrol sensor_msgs)
ament_target_dependencies(go_to_pose_action rclcpp robot_patrol rclcpp_action geometry_msgs nav_msgs)

# Install targets
install(TARGETS patrol DESTINATION lib/${PROJECT_NAME})
install(TARGETS direction_service DESTINATION lib/${PROJECT_NAME})
install(TARGETS test_service DESTINATION lib/${PROJECT_NAME})
install(TARGETS patrol_with_service DESTINATION lib/${PROJECT_NAME})
install(TARGETS go_to_pose_action DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
ament_export_dependencies(ament_cmake rclcpp sensor_msgs geometry_msgs) 

ament_package()
