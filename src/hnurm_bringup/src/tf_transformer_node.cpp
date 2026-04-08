#include "hnurm_bringup/tf_transformer_node.hpp"
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <cmath>

using namespace std::chrono_literals;

namespace hnurm
{
    TfTransformer::TfTransformer(const rclcpp::NodeOptions &options)
        : Node("TfTransformer", options)
    {

        RCLCPP_INFO(get_logger(), "TfTransformer is running");
        std::vector<std::string> frame_names = {
            "lidar_to_base","back_camera_to_base"
        };
        std::map<std::string, extrinsic> transforms;

        // 获取各个传感器的变换参数
        for(const auto & frame : frame_names) {
            transforms[frame] = getTransformParams(frame);
        }

        lidar_to_base = transforms["lidar_to_base"];
        camera_to_joint = transforms["camera_to_joint"];
        joint_to_base = transforms["joint_to_base"];
        back_camera_to_base = transforms["back_camera_to_base"];

        // TF 初始化
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // 里程计订阅
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 
            rclcpp::SensorDataQoS(), 
            std::bind(&TfTransformer::odom_callback, this, std::placeholders::_1)
        );
        init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            rclcpp::SensorDataQoS(),
            std::bind(&TfTransformer::initial_pose_callback, this, std::placeholders::_1));

        // 发布转换后的里程计
        odom_pub_  
         = this->create_publisher<nav_msgs::msg::Odometry>(
            "Odometry_transformed", 
            rclcpp::SensorDataQoS()
        );   

        // 订阅串口接收数据
        recv_data_sub_ 
         = this->create_subscription<hnurm_interfaces::msg::VisionRecvData>(
            "vision_recv_data", 
            rclcpp::SensorDataQoS(), 
            std::bind(&TfTransformer::recv_data_callback, this, std::placeholders::_1)
        );
        
        // 定时器初始化
        timer_tf_ = this->create_wall_timer(100ms, std::bind(&TfTransformer::timer_callback, this));

    }
    
    // 获取变换参数
    TfTransformer::extrinsic TfTransformer::getTransformParams(const std::string & ns)
    {
        extrinsic t;
        this->get_parameter(ns + ".x", t.x);
        this->get_parameter(ns + ".y", t.y);
        this->get_parameter(ns + ".z", t.z);
        this->get_parameter(ns + ".roll", t.roll);
        this->get_parameter(ns + ".pitch", t.pitch);
        this->get_parameter(ns + ".yaw", t.yaw);
        return t;
    }

    // 初始化位姿回调
    void TfTransformer::initial_pose_callback( const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        initial_pose_guess_.translation.x = msg->pose.pose.position.x;
        initial_pose_guess_.translation.y = msg->pose.pose.position.y;
        initial_pose_guess_.translation.z = msg->pose.pose.position.z;
        initial_pose_guess_.rotation.w = msg->pose.pose.orientation.w;
        initial_pose_guess_.rotation.x = msg->pose.pose.orientation.x;
        initial_pose_guess_.rotation.y = msg->pose.pose.orientation.y;
        initial_pose_guess_.rotation.z = msg->pose.pose.orientation.z;
        is_initial_pose_get_ = true;
    }

    // 串口数据回调
    void TfTransformer::recv_data_callback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg)
    {
        rpy_.roll = msg->roll;
        rpy_.pitch = msg->pitch;
        rpy_.yaw = msg->yaw;
        if(msg->self_color.data == 1) 
        { 
            self_color = "RED";
            init_trans_.translation.x = 0.0;
            init_trans_.translation.y = 0.0;
            init_trans_.translation.z = 0.0;
            init_trans_.rotation.x = 0.0;
            init_trans_.rotation.y = 0.0;
            init_trans_.rotation.z = 0.0;
            init_trans_.rotation.w = 1.0;
        }
        else  {
            self_color = "BLUE";
            init_trans_.translation.x = 0.0;
            init_trans_.translation.y = 0.0;
            init_trans_.translation.z = 0.0;
            init_trans_.rotation.x = 0.0;
            init_trans_.rotation.y = 0.0;
            init_trans_.rotation.z = 0.0;
            init_trans_.rotation.w = 1.0;
        }
        is_self_color_set_ = true;
    }

    // 定时器回调
    void TfTransformer::timer_callback()
    {
        // 创建 TransformStamped 消息
        rclcpp::Time now = this->now();
        geometry_msgs::msg::TransformStamped static_transform;
        geometry_msgs::msg::TransformStamped transformStamped;
        tf2::Quaternion q;

        // map -> odom : 动态发布（支持重定位）
        if(is_self_color_set_)
        {
            geometry_msgs::msg::Transform transfrom_;
            if (!is_initial_pose_get_)
            {
                transfrom_ = init_trans_;
            }
            else
                transfrom_ = initial_pose_guess_;

            transformStamped.header.stamp = now;
            transformStamped.header.frame_id = "map";
            transformStamped.child_frame_id = "odom";
            transformStamped.transform.translation.x = transfrom_.translation.x;
            transformStamped.transform.translation.y = transfrom_.translation.y;
            transformStamped.transform.translation.z = transfrom_.translation.z;
            transformStamped.transform.rotation.x = transfrom_.rotation.x;
            transformStamped.transform.rotation.y = transfrom_.rotation.y;
            transformStamped.transform.rotation.z = transfrom_.rotation.z;
            transformStamped.transform.rotation.w = transfrom_.rotation.w;
            tf_broadcaster_->sendTransform(transformStamped);
        }

        if(is_finished_static_transform_) return;
        /*tf tree
            map
            └──► odom
                  └──► base_link ──► lidar_link ──► livox_frame
                         │
                         └──► base_footprint ──► gimbal_link ──► camera_link ──► camera_optical_frame
                                  │
                                  └──► back_camera
        */
        
        static_transform.header.frame_id = "base_footprint";   // 父坐标系
        static_transform.child_frame_id = "lidar_link";    // 子坐标系
        static_transform.transform.translation.x = lidar_to_base.x;
        static_transform.transform.translation.y = lidar_to_base.y;
        static_transform.transform.translation.z = lidar_to_base.z;
        q.setRPY(0.0, 0.0, 0.0);  // 旋转角度（单位：弧度）
        static_transform.transform.rotation.x = q.x();
        static_transform.transform.rotation.y = q.y();
        static_transform.transform.rotation.z = q.z();
        static_transform.transform.rotation.w = q.w();
        static_broadcaster_->sendTransform(static_transform);

        // lidar_link -> livox_frame : 静态变换，Livox雷达坐标系
        static_transform.header.frame_id = "lidar_link";   // 父坐标系
        static_transform.child_frame_id = "livox_frame";    // 子坐标系
        static_transform.transform.translation.x = 0.0;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.0;
        q.setRPY(0.0, 0.0, 0.0);  // 旋转角度（单位：弧度）
        static_transform.transform.rotation.x = q.x();
        static_transform.transform.rotation.y = q.y();
        static_transform.transform.rotation.z = q.z();
        static_transform.transform.rotation.w = q.w();
        static_broadcaster_->sendTransform(static_transform);

        // base_link -> base_footprint : 静态变换，底盘坐标系
        static_transform.header.frame_id = "base_link";   // 父坐标系
        static_transform.child_frame_id = "base_footprint";    // 子坐标系
        static_transform.transform.translation.x = -lidar_to_base.x;
        static_transform.transform.translation.y = -lidar_to_base.y;
        static_transform.transform.translation.z = -lidar_to_base.z;
        q.setRPY(0.0, 0.0, 0.0);  // 旋转角度（单位：弧度）
        static_transform.transform.rotation.x = q.x();
        static_transform.transform.rotation.y = q.y();
        static_transform.transform.rotation.z = q.z();
        static_transform.transform.rotation.w = q.w();
        static_broadcaster_->sendTransform(static_transform);

        // base_footprint -> back_camera : 静态变换，后视摄像头坐标系
        static_transform.header.frame_id = "base_footprint";   // 父坐标系
        static_transform.child_frame_id = "back_camera";    // 子坐标系
        static_transform.transform.translation.x = back_camera_to_base.x;
        static_transform.transform.translation.y = back_camera_to_base.y;
        static_transform.transform.translation.z = back_camera_to_base.z;
        q.setRPY(0.0, 0.0,-M_PI);  // 旋转角度（单位：弧度）
        static_transform.transform.rotation.x = q.x();
        static_transform.transform.rotation.y = q.y();
        static_transform.transform.rotation.z = q.z();
        static_transform.transform.rotation.w = q.w();
        static_broadcaster_->sendTransform(static_transform);

       

        is_finished_static_transform_ = true;

    }

    // 里程计回调
    void TfTransformer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        nav_msgs::msg::Odometry odom_transformed;       // 要发布的odom->base_footprint变换信息
        geometry_msgs::msg::TransformStamped transform; // 存储lidar_link->base_footprint静态变换信息
        geometry_msgs::msg::Pose transformedPose;       // 存储odom->base_footprint变换位姿
        geometry_msgs::msg::Pose odom2base_link;        // 来自里程记的odom->base_link位姿
        odom_transformed.twist = msg->twist;
        odom_transformed.header = msg->header;
        odom_transformed.pose.covariance = msg->pose.covariance;
        odom2base_link = msg->pose.pose;
        
        try {
            transform = tf_buffer_->lookupTransform("lidar_link", "base_footprint", rclcpp::Time(0), 0.1s);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s. Waiting for transform...", ex.what());
            return;  // 如果转换查找失败，则退出该回调
        }
        
        try {
            // 将odom->base_link位姿转换为odom->base_footprint
            tf2::doTransform(odom2base_link, transformedPose, transform);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
            return;
        }
        // 发布转换后的里程计
        odom_transformed.header.frame_id = "odom";
        odom_transformed.child_frame_id = "base_footprint";
        odom_transformed.pose.pose = transformedPose;
        odom_pub_->publish(odom_transformed);


        geometry_msgs::msg::TransformStamped odom2base_link_transform;
        odom2base_link_transform.header.stamp = msg->header.stamp;
        odom2base_link_transform.header.frame_id = "odom";
        odom2base_link_transform.child_frame_id = "base_link";
        odom2base_link_transform.transform.translation.x = msg->pose.pose.position.x;
        odom2base_link_transform.transform.translation.y = msg->pose.pose.position.y;
        odom2base_link_transform.transform.translation.z = msg->pose.pose.position.z;
        odom2base_link_transform.transform.rotation.x = msg->pose.pose.orientation.x;
        odom2base_link_transform.transform.rotation.y = msg->pose.pose.orientation.y;
        odom2base_link_transform.transform.rotation.z = msg->pose.pose.orientation.z;
        odom2base_link_transform.transform.rotation.w = msg->pose.pose.orientation.w;
        tf_broadcaster_->sendTransform(odom2base_link_transform);
    }

   

}  // namespace hnurm

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<hnurm::TfTransformer>(options);
    // rclcpp::spin(node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}
