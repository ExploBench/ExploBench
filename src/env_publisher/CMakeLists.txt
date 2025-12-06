cmake_minimum_required(VERSION 3.8)
project(env_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav_msgs REQUIRED)

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

add_executable(ply_publisher_node 
  src/ply_publisher.cpp 
  src/ply_publisher_node.cpp
)

add_executable(pcd_publisher_node 
  src/pcd_publisher.cpp 
  src/pcd_publisher_node.cpp
)

target_include_directories(ply_publisher_node PUBLIC
  ${PCL_INCLUDE_DIRS}
)

target_include_directories(pcd_publisher_node PUBLIC
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(ply_publisher_node
  ${PCL_LIBRARIES}
)

target_link_libraries(pcd_publisher_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(ply_publisher_node 
  sensor_msgs
  rclcpp
  pcl_conversions
  nav_msgs
)

ament_target_dependencies(pcd_publisher_node 
  sensor_msgs
  rclcpp
  pcl_conversions
  nav_msgs
)

install(DIRECTORY env
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ply_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  pcd_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)
