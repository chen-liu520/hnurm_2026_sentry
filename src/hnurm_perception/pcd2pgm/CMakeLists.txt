cmake_minimum_required(VERSION 3.10)
project(pcd2pgm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  rclcpp 
  sensor_msgs 
  tf2 
  tf2_ros 
  tf2_eigen
  pcl_conversions
  geometry_msgs
  pcl_ros
  nav_msgs
)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)

if(OpenMP_CXX_FOUND)
  add_definitions(-DOPENMP_AVAILABLE)  
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


ament_auto_add_library(pcd2pgm
        src/pcd2pgm_node.cpp
)



ament_auto_add_executable(pcd2pgm_node src/main.cpp src/pcd2pgm_node.cpp)
# link library

target_include_directories(pcd2pgm_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pcd2pgm_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  rclcpp::rclcpp
  OpenMP::OpenMP_CXX
)

ament_auto_package(
        INSTALL_TO_SHARE
        launch
        params
)
