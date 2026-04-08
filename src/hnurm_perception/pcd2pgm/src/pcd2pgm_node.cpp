#include "pcd2pgm/pcd2pgm_node.hpp"

namespace hnurm {

Pcd2pgmNode::Pcd2pgmNode(const rclcpp::NodeOptions &options)
        : Node("Pcd2pgmNode", options) 
{
  RCLCPP_INFO(get_logger(), "Pcd2pgmNode is running");

  pcd_file_ = this->declare_parameter("pcd_file","/home/rm/nav/src/hnunavigation_-ros2/hnurm_perception/PCD/all_raw_points.pcd");
  start_height_ = this->declare_parameter<double>("start_height",0.2);
  end_height_ = this->declare_parameter<double>("end_height",2.0);
  resolution_ = this->declare_parameter<double>("resolution",0.05);

  // init pointcloud pointer
  pcd_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  filtered_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  //load pcd
  load_pcd_map(pcd_file_);

  //publishers
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_pcd_map", 10);
  pointcloud_pub_segmented_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcd_segmented", 10);
  grid_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Pcd2pgmNode::timer_callback, this));

}

void Pcd2pgmNode::update_params()
{
    start_height_ = this->get_parameter("start_height").get_value<double>();
    end_height_ = this->get_parameter("end_height").get_value<double>();
    resolution_ = this->get_parameter("resolution").get_value<double>();
}

void Pcd2pgmNode::load_pcd_map(const std::string& map_path){

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *pcd_map_) == -1) {
      RCLCPP_ERROR(get_logger(), "Failed to load PCD map: %s", map_path.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Loaded PCD map with %ld points", pcd_map_->size());

    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*pcd_map_, output_cloud);
    output_cloud.header.frame_id = "map";
    output_cloud.header.stamp = this->now();
    cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>(output_cloud);
  }
  
void Pcd2pgmNode::timer_callback()
{
    if(cloud_)
    {
        pointcloud_pub_->publish(*cloud_);
    }
    update_params();
    // 创建直通滤波器对象
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcd_map_);
    pass.setFilterFieldName("z");     
    pass.setFilterLimits(start_height_, end_height_);   
    pass.filter(*filtered_cloud);

    pcl::toROSMsg(*filtered_cloud, output_cloud);
    output_cloud.header.frame_id = "map";
    output_cloud.header.stamp = this->now();

    cloud_segmented_ = std::make_shared<sensor_msgs::msg::PointCloud2>(output_cloud);

    // 处理点云并生成栅格
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    convert_to_grid(*filtered_cloud, occupancy_grid);
    grid_map_pub_->publish(occupancy_grid);
    if (cloud_segmented_)
    {
        pointcloud_pub_segmented_->publish(*cloud_segmented_);

    }
    
}

void Pcd2pgmNode::convert_to_grid(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    nav_msgs::msg::OccupancyGrid& grid
) {
    float x_min = std::numeric_limits<float>::max();
    float x_max = -x_min;
    float y_min = x_min, y_max = x_max;
    float z_min_ = start_height_;
    float z_max_ = end_height_;
    for (const auto& pt : cloud.points) {
        if (pt.z < z_min_ || pt.z > z_max_) continue; // 直通滤波
        x_min = std::min(x_min, pt.x);
        x_max = std::max(x_max, pt.x);
        y_min = std::min(y_min, pt.y);
        y_max = std::max(y_max, pt.y);
    }
    
    // 设置栅格元数据
    grid.header.stamp = this->now();
    grid.header.frame_id = "map"; // 确保与TF树一致
    grid.info.resolution = resolution_;
    grid.info.width = static_cast<uint32_t>((x_max - x_min) / resolution_) + 1;
    grid.info.height = static_cast<uint32_t>((y_max - y_min) / resolution_) + 1;
    grid.info.origin.position.x = x_min;
    grid.info.origin.position.y = y_min;
    grid.info.origin.orientation.w = 1.0; // 无旋转
    
    // 初始化栅格数据（-1表示未知，0~100表示占据概率）
    grid.data.resize(grid.info.width * grid.info.height, -1);
    
    // 将点云投影到栅格
    for (const auto& pt : cloud.points) {
        if (pt.z < z_min_ || pt.z > z_max_) continue;

        // 使用 uint32_t 替代 int
        uint32_t col = static_cast<uint32_t>((pt.x - x_min) / resolution_);
        uint32_t row = static_cast<uint32_t>((pt.y- y_min) / resolution_);

        // 直接比较无符号整数（无需 >=0 检查）
        if (row < grid.info.height && col < grid.info.width) {
            uint32_t index = row * grid.info.width + col;
            grid.data[index] = 100; 
        }
    }
}





}