cmake_minimum_required(VERSION 3.10)
project(tspplanner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Find OR-Tools using pkg-config
find_package(PkgConfig)
if(PkgConfig_FOUND)
  pkg_check_modules(ORTOOLS ortools)
  if(ORTOOLS_FOUND)
    add_definitions(-DORTOOLS_FOUND)
    message(STATUS "OR-Tools found")
  else()
    message(WARNING "OR-Tools not found. Please install it: sudo apt-get install libortools-dev")
    message(WARNING "TSP solver will use fallback method")
  endif()
else()
  message(WARNING "pkg-config not found, cannot check for OR-Tools")
endif()

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

add_executable(tspplanner_node src/tspplanner.cpp src/tspplanner_node.cpp)

ament_target_dependencies(tspplanner_node
  rclcpp
  std_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs
  sensor_msgs
)

if(ORTOOLS_FOUND)
  target_link_libraries(tspplanner_node ${ORTOOLS_LIBRARIES})
  target_include_directories(tspplanner_node PRIVATE ${ORTOOLS_INCLUDE_DIRS})
endif()

install(TARGETS
  tspplanner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

