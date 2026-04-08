#pragma once

#include "hnurm_interfaces/msg/target.hpp"
#include "hnurm_interfaces/msg/vision_recv_data.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include <mutex>
#include <thread>
#include <vector>
#include <tf2/LinearMath/Quaternion.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

//#include <visualization_msgs/msg/marker_array.hpp>
//#include <visualization_msgs/msg/marker.hpp>

namespace hnurm
{
    
class TfTransformer : public rclcpp::Node
{
public:
    explicit TfTransformer(const rclcpp::NodeOptions &options);

    ~TfTransformer()
    {
        RCLCPP_INFO(get_logger(), "TfTransformer destroyed");
    }

    TfTransformer(const TfTransformer &)            = delete;
    TfTransformer &operator=(const TfTransformer &) = delete;
    TfTransformer(TfTransformer &&)                 = delete;
    TfTransformer &operator=(TfTransformer &&)      = delete;
    struct extrinsic
    {
        float x,y,z,roll,pitch,yaw;
    };

    struct current_rpy
    {
        float roll,pitch,yaw;
    };
    

    extrinsic lidar_to_base,camera_to_joint,joint_to_base,back_camera_to_base;
    current_rpy rpy_{0.0,0.0,0.0};
    bool is_self_color_set_ = false;
    std::string self_color = "RED";   //default blue
    geometry_msgs::msg::Transform init_trans_;  //initial pose
    geometry_msgs::msg::Transform initial_pose_guess_;  //initial pose callback
    bool is_initial_pose_get_ = false;
    
protected:
    std::shared_ptr<TfTransformer> shared_from_this()
    {
        return std::static_pointer_cast<TfTransformer>(std::shared_ptr<rclcpp::Node>(this));
    }
private:

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<hnurm_interfaces::msg::VisionRecvData>::SharedPtr recv_data_sub_;
    //rclcpp::Subscription<hnurm_interfaces::msg::ArmorArray>::SharedPtr back_armor_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;

    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr  twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr     twist_pub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr back_target_; 
  
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_tf_;
    rclcpp::TimerBase::SharedPtr timer_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    void recv_data_callback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    float calculate_theta(const float center);
    void timer_callback();
    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg );
    extrinsic getTransformParams(const std::string & ns);
};
}