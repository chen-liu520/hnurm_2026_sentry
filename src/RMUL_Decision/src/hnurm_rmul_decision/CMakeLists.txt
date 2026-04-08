cmake_minimum_required(VERSION 3.8)
project(hnurm_ul_decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(hnurm_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Include directories
include_directories(include/hnurm_decision_gesture)

# Build plugins
add_library(pub_robot_status_plugin SHARED plugins/action/PubRobotStatus.cpp)
ament_target_dependencies(pub_robot_status_plugin 
  rclcpp 
  behaviortree_cpp_v3 
  std_msgs 
  geometry_msgs 
  hnurm_interfaces 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
  std_srvs
  visualization_msgs
)

add_library(game_start_plugin SHARED plugins/condition/GameStart.cpp)
ament_target_dependencies(game_start_plugin 
  rclcpp 
  behaviortree_cpp_v3
  hnurm_interfaces
)

add_library(status_plugin SHARED plugins/condition/Status.cpp)
ament_target_dependencies(status_plugin 
  rclcpp 
  behaviortree_cpp_v3
  hnurm_interfaces
)

# Build main executable
add_executable(decision_node src/decision.cpp)
ament_target_dependencies(decision_node 
  rclcpp 
  behaviortree_cpp_v3 
  ament_index_cpp 
  std_msgs 
  geometry_msgs 
  hnurm_interfaces
  std_srvs
  tf2_geometry_msgs
  visualization_msgs
)

# Link against plugins
target_link_libraries(decision_node 
  pub_robot_status_plugin
  game_start_plugin
  status_plugin
)

# Set RPATH so decision_node can find shared libraries at runtime
set_target_properties(decision_node PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# Install targets
install(TARGETS
  decision_node
  pub_robot_status_plugin
  game_start_plugin
  status_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Install param files
install(DIRECTORY param/ DESTINATION share/${PROJECT_NAME}/param)

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