#pragma once

#include "hnurm_interfaces/msg/vision_recv_data.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

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

        TfTransformer(const TfTransformer &) = delete;
        TfTransformer &operator=(const TfTransformer &) = delete;
        TfTransformer(TfTransformer &&) = delete;
        TfTransformer &operator=(TfTransformer &&) = delete;

        struct extrinsic
        {
            float x, y, z, roll, pitch, yaw;
        };

        extrinsic lidar_to_base, back_camera_to_base;
        bool is_self_color_set_ = false;
        geometry_msgs::msg::Transform init_trans_;
        geometry_msgs::msg::Transform initial_pose_guess_;
        bool is_finished_static_transform_ = false;
        bool is_initial_pose_get_ = false;
        bool is_relocating = false;

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<hnurm_interfaces::msg::VisionRecvData>::SharedPtr recv_data_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr relocalization_sub_;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

        rclcpp::TimerBase::SharedPtr timer_tf_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

        void recv_data_callback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg);
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void relocalization_state_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void timer_callback();
        void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
        extrinsic getTransformParams(const std::string &ns);
    };

} // namespace hnurm
