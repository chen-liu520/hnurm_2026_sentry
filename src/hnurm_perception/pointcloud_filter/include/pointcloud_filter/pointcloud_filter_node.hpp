#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp> 
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <hnurm_interfaces/msg/zone_end_point2_d.hpp>
#include <hnurm_interfaces/msg/special_area.hpp>
#include <hnurm_interfaces/msg/area.hpp>
#include <hnurm_interfaces/msg/type.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include "nav_msgs/msg/odometry.hpp"

namespace hnurm {



class PointCloudNode : public rclcpp::Node{
  public:
    // void run();
    explicit PointCloudNode(const rclcpp::NodeOptions &options);
    ~PointCloudNode()
    {
        RCLCPP_INFO(get_logger(), "PointCloudNode destroyed");
    }
    PointCloudNode(const PointCloudNode &)            = delete;
    PointCloudNode &operator=(const PointCloudNode &) = delete;
    PointCloudNode(PointCloudNode &&)                 = delete;
    PointCloudNode &operator=(PointCloudNode &&)      = delete;


  private:
    void pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void laser_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
    void spa_callback(const hnurm_interfaces::msg::SpecialArea::SharedPtr msg);
    hnurm_interfaces::msg::SpecialArea transform_SpecialArea(const geometry_msgs::msg::TransformStamped& transform,const hnurm_interfaces::msg::SpecialArea& spa);
    bool is_in_SpecialArea(float x,float y,const hnurm_interfaces::msg::SpecialArea& area);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void is_in_SpecialArea_callback(const std_msgs::msg::Bool::SharedPtr msg);
    // void pointcloud_to_laserscan(pcl::PointCloud<pcl::PointXYZI>& cloud);
    sensor_msgs::msg::PointCloud2 create_empty_pointcloud(const sensor_msgs::msg::PointCloud2& msg );
  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<hnurm_interfaces::msg::SpecialArea>::SharedPtr spa_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr in_special_area_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr test_laser_pub_;
    rclcpp::Publisher<hnurm_interfaces::msg::SpecialArea>::SharedPtr tfed_spa_pub_;


    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


    std::string base_frame_;
    std::string lidar_frame_;
    std::string laser_frame_;
    std::string lidar_topic_;
    std::string output_lidar_topic_;
    std::string laser_topic_;
    std::string output_laser_topic_;
    std::string odom_topic_;
    float sensor_height_;
    float robot_radius_;
    std::string filter_topic_;

    hnurm_interfaces::msg::SpecialArea current_spa_;

        /*
    current_spa_(map) -> temp_tfed_spa_(laser_link or lidar_link))
    only filter the closest special area
    */
    float current_sensor_height;
    bool use_filter = false;

  protected:
    std::shared_ptr<PointCloudNode> shared_from_this(){
      return std::static_pointer_cast<PointCloudNode>(rclcpp::Node::shared_from_this());
    }


};
}