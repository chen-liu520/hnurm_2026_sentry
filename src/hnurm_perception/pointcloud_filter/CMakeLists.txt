cmake_minimum_required(VERSION 3.5)
project(pointcloud_filter)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 自动查找 ROS 依赖
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED 
  rclcpp 
  sensor_msgs 
  tf2 
  tf2_ros 
  tf2_eigen
  pcl_conversions
  hnurm_interfaces
  nav_msgs
  message_filters
)

# 显式查找 PCL 并指定组件
find_package(PCL REQUIRED COMPONENTS common filters)

# 添加可执行文件
ament_auto_add_executable(pointcloud_filter_node src/main.cpp src/pointcloud_filter_node.cpp)

# 链接库
target_include_directories(pointcloud_filter_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pointcloud_filter_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  rclcpp::rclcpp
)

# 安装和打包
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  params
)