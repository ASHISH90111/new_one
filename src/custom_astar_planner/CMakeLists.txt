cmake_minimum_required(VERSION 3.8)
project(custom_astar_planner)

# Compiler options (good practice)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===============================
# Find dependencies
# ===============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ===============================
# Include directories
# ===============================
include_directories(
  include
)

# ===============================
# Build the planner plugin library
# ===============================
add_library(custom_astar_planner SHARED
  src/astar_planner.cpp
  src/astar.cpp
)

ament_target_dependencies(custom_astar_planner
  rclcpp
  nav2_core
  nav2_costmap_2d
  pluginlib
  nav_msgs
  geometry_msgs
  tf2_ros
)

# ===============================
# Export plugin description
# ===============================
pluginlib_export_plugin_description_file(
  nav2_core
  plugin.xml
)

# ===============================
# Install targets
# ===============================
install(
  TARGETS custom_astar_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# ===============================
# Testing (unchanged)
# ===============================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ===============================
# Package export
# ===============================
ament_package()
