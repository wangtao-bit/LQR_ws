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
)
target_link_libraries(lqr_tracker_node Eigen3::Eigen)

add_executable(rviz_path_builder_node
  src/rviz_path_builder_node.cpp
)
ament_target_dependencies(rviz_path_builder_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
)

install(TARGETS
  lqr_tracker_node
  rviz_path_builder_node
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
