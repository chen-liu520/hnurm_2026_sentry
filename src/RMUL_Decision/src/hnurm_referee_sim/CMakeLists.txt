cmake_minimum_required(VERSION 3.8)
project(hnurm_referee_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hnurm_interfaces REQUIRED)

# 添加可执行文件
add_executable(referee_sim_node src/referee_sim_node.cpp)
ament_target_dependencies(referee_sim_node
  rclcpp
  std_msgs
  hnurm_interfaces
)

# 安装目标
install(TARGETS
  referee_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
