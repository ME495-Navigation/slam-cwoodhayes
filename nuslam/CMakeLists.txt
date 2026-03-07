cmake_minimum_required(VERSION 3.20)
project(nuslam)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(catch_ros2 REQUIRED)
find_package(turtle_control REQUIRED)

# Add nodes
add_executable(slam_node src/slam.cpp)
target_link_libraries(slam_node PUBLIC
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${nuturtlebot_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${turtle_control_TARGETS}
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  turtlelib::turtlelib
  ${cpp_typesupport_target}
)

# Install nodes
install(TARGETS slam_node DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   include(CTest)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()

#   add_executable(slam_test_node
#     test/slam_test.cpp
#   )

#   target_link_libraries(slam_test_node PUBLIC
#     catch_ros2::catch_ros2_with_node_main
#     ${std_msgs_TARGETS}
#     ${std_srvs_TARGETS}
#     ${geometry_msgs_TARGETS}
#     ${visualization_msgs_TARGETS}
#     ${nuturtlebot_msgs_TARGETS}
#     ${nav_msgs_TARGETS}
#     rclcpp::rclcpp
#     tf2_ros::tf2_ros
#     turtlelib::turtlelib
#     catch_ros2::catch_ros2
#     ${cpp_typesupport_target}
#   )

#   install(TARGETS slam_test_node DESTINATION lib/${PROJECT_NAME})

#   # makes sure int test runs when running colcon test
#   catch_ros2_add_integration_test(slam_test
#     LAUNCH_FILE slam_test.launch.xml
#   )

#   # Install test launch files.
#   install(DIRECTORY
#     test
#     DESTINATION share/${PROJECT_NAME}/
#   )
# endif()

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
