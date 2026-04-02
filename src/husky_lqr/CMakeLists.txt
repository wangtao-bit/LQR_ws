cmake_minimum_required(VERSION 3.8)
project(husky_lqr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(lqr_tracker_node
  src/lqr_tracker_node.cpp
  src/unified_lqr_solver.cpp
)
ament_target_dependencies(lqr_tracker_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  visualization_msgs
)
target_link_libraries(lqr_tracker_node Eigen3::Eigen)

add_executable(bow_coverage_path_publisher
  src/bow_coverage_path_publisher.cpp
)
ament_target_dependencies(bow_coverage_path_publisher
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
)

add_executable(tracking_metrics_node
  src/tracking_metrics_node.cpp
)
ament_target_dependencies(tracking_metrics_node
  rclcpp
  nav_msgs
)

install(TARGETS
  lqr_tracker_node
  bow_coverage_path_publisher
  tracking_metrics_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
