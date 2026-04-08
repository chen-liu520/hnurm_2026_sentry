#pragma once
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <random>




#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp> 
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

//eigen libs
#include <Eigen/Geometry>
#include <Eigen/Core>

//pcl libs
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <mutex>
namespace hnurm {

using QuatroPointType = pcl::PointXYZ; //can be changed


class Pcd2pgmNode : public rclcpp::Node{
  public:
    // void run();
    explicit Pcd2pgmNode(const rclcpp::NodeOptions &options);
    ~Pcd2pgmNode()
    {
        RCLCPP_INFO(get_logger(), "Pcd2pgmNode destroyed");
    }
    Pcd2pgmNode(const Pcd2pgmNode &)            = delete;
    Pcd2pgmNode &operator=(const Pcd2pgmNode &) = delete;
    Pcd2pgmNode(Pcd2pgmNode &&)                 = delete;
    Pcd2pgmNode &operator=(Pcd2pgmNode &&)      = delete;


  private://functions
    void update_params();
    void timer_callback();
    void load_pcd_map(const std::string& map_path);
    void convert_to_grid(const pcl::PointCloud<pcl::PointXYZ>& cloud,nav_msgs::msg::OccupancyGrid& grid); 


  private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_segmented_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;


    //variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_map_;   
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;  

    sensor_msgs::msg::PointCloud2::SharedPtr  cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr  cloud_segmented_;
    //params 

    std::string pcd_file_;
    double start_height_;
    double end_height_;
    double resolution_;

    std::string output_pgm_;

  protected:
    std::shared_ptr<Pcd2pgmNode> shared_from_this(){
      return std::static_pointer_cast<Pcd2pgmNode>(rclcpp::Node::shared_from_this());
    }
};



}