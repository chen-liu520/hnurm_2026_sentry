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
            "imu_to_joint", "laser_to_base", "lidar_to_base",
            "camera_to_joint","joint_to_base","back_camera_to_base"
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

        // *** 订阅与发布器初始化 ***

        // odom_sub_ 
        //  = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "/LIVO2/imu_propagate", 
        //     rclcpp::SensorDataQoS(), 
        //     std::bind(&TfTransformer::odom_callback, this, std::placeholders::_1)
        // );

        // 里程计订阅
        odom_sub_
         = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 
            rclcpp::SensorDataQoS(), 
            std::bind(&TfTransformer::odom_callback, this, std::placeholders::_1)
        );

        // 发布转换后的里程计
        odom_pub_  //adpat to this , choose the right qos
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

        // 后视相机 armor 订阅
        // back_armor_sub_
        // = this->create_subscription<hnurm_interfaces::msg::Armors>(
        //     "/back_camera/back_armor", 
        //     rclcpp::SensorDataQoS(), 
        //     std::bind(&TfTransformer::back_camera_callback, this, std::placeholders::_1)
        // );
        
        // 速度旋转订阅（小陀螺？）
        twist_sub_
        = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 
            rclcpp::SensorDataQoS(), 
            std::bind(&TfTransformer::twist_callback, this, std::placeholders::_1)
        );

        // 初始化位姿订阅
        init_pose_sub_
         = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            rclcpp::SensorDataQoS(),
            std::bind(&TfTransformer::initial_pose_callback, this, std::placeholders::_1)
        );

        // 发布虚拟速度
        twist_pub_
         = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel_fake",
            rclcpp::ServicesQoS()
        );  

        // 后视目标发布
        back_target_
         = this->create_publisher<std_msgs::msg::Float32>(
            "back_target", 
            rclcpp::SensorDataQoS()
        ); 

        // marker_publisher_
        //  = this->create_publisher<visualization_msgs::msg::Marker>(
        //     "back_target",
        //     rclcpp::SensorDataQoS()
        // );
        
        // 定时器初始化
        timer_tf_ = this->create_wall_timer(10ms, std::bind(&TfTransformer::timer_callback, this));

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

    // 计算目标偏转角度
    float TfTransformer::calculate_theta(const float center) {
        // 960 是图像中心，1920 是图像宽度
            
        if (center < 0 || center > 1920.0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid center value: %f", center);
            return 0.0f;
        }
        float delta = (center < 960.0) 
            ? (960.0 - center) 
            : (center - 960.0);

        return atan2((delta*1.732/960.0)*3.0,3.0)*180.0/M_PI;
        // return std::atan((delta*0.0072) / 4.0)*180.0/M_PI;
    }

    // 初始化位姿回调
    void TfTransformer::initial_pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
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
        // std::lock_guard<std::mutex> lock(mtx);

        // // send target info over uart
        // send_target(tmp_target, last_recv_array_.recv_uart);
        /*tf tree
                                                        />laser_link          /> imu_link            
            map -> odom -> base_link -> base_footprint -> joint_link ---------> back_camera      
                                                        \>lidar_link          \> camera_link   
                                                        \>imu_init ---> camera_init                                
        */
        rclcpp::Time now = this->now();

        // 创建 TransformStamped 消息
        geometry_msgs::msg::TransformStamped transformStamped;

        // // joint_link -> imu_link : 固定yaw旋转-180°
        // transformStamped.header.stamp = now;
        // transformStamped.header.frame_id = "joint_link";  // 转轴坐标系
        // transformStamped.child_frame_id = "imu_link";   // IMU 坐标系
        // transformStamped.transform.translation.x = imu_to_joint.x;
        // transformStamped.transform.translation.y = imu_to_joint.y;
        // transformStamped.transform.translation.z = imu_to_joint.z;
        tf2::Quaternion q;
        // q.setRPY(0,0,-M_PI);
        // transformStamped.transform.rotation.x = q.x();
        // transformStamped.transform.rotation.y = q.y();
        // transformStamped.transform.rotation.z = q.z();
        // transformStamped.transform.rotation.w = q.w();
        // tf_broadcaster_->sendTransform(transformStamped);

        // odom -> hip_imu_footprint : 使用串口接收的roll/pitch/yaw
        // transformStamped.header.frame_id = "odom";  // 初始化的里程计原点
        // transformStamped.child_frame_id = "hip_imu_footprint";   // IMU 坐标系
        // transformStamped.transform.translation.x = 0;
        // transformStamped.transform.translation.y = 0;
        // transformStamped.transform.translation.z = 0;
        // q.setRPY(rpy_.roll*M_PI/180.0,rpy_.pitch*M_PI/180.0,rpy_.yaw*M_PI/180.0);
        // transformStamped.transform.rotation.x = q.x();
        // transformStamped.transform.rotation.y = q.y();
        // transformStamped.transform.rotation.z = q.z();
        // transformStamped.transform.rotation.w = q.w();
        // tf_broadcaster_->sendTransform(transformStamped);

        // base_footprint -> joint_link : 实时补偿pitch角
        // transformStamped.header.frame_id = "base_footprint";  // lidar 
        // transformStamped.child_frame_id = "joint_link";   //joint
        // transformStamped.transform.translation.x = joint_to_base.x;
        // transformStamped.transform.translation.y = joint_to_base.y;
        // transformStamped.transform.translation.z = joint_to_base.z;
        // q.setRPY(0, -rpy_.pitch*M_PI/180.0, 0);
        // transformStamped.transform.rotation.x = q.x();
        // transformStamped.transform.rotation.y = q.y();
        // transformStamped.transform.rotation.z = q.z();
        // transformStamped.transform.rotation.w = q.w();
        // tf_broadcaster_->sendTransform(transformStamped);

        // base_footprint -> laser_link : 静态变换，激光相对底盘
        // geometry_msgs::msg::TransformStamped static_transform;
        // static_transform.header.frame_id = "base_footprint";   // 父坐标系
        // static_transform.child_frame_id = "laser_link";    // 子坐标系
        // static_transform.transform.translation.x = laser_to_base.x;
        // static_transform.transform.translation.y = laser_to_base.y;
        // static_transform.transform.translation.z = laser_to_base.z;
        // q.setRPY(0.0, 0.0, 0.0);  // 旋转角度（单位：弧度）
        // static_transform.transform.rotation.x = q.x();
        // static_transform.transform.rotation.y = q.y();
        // static_transform.transform.rotation.z = q.z();
        // static_transform.transform.rotation.w = q.w();
        // static_broadcaster_->sendTransform(static_transform);

        // base_footprint -> lidar_link : 静态变换，mid360相对底盘
        geometry_msgs::msg::TransformStamped static_transform;
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

        // // joint_link -> camera_link : 静态变换，前视相机相对转轴
        // static_transform.header.frame_id = "joint_link";   // 父坐标系
        // static_transform.child_frame_id = "camera_link";    // 子坐标系
        // static_transform.transform.translation.x = camera_to_joint.x;
        // static_transform.transform.translation.y = camera_to_joint.y;
        // static_transform.transform.translation.z = camera_to_joint.z;
        // q.setRPY(0.0, 0.0, 0.0);  // 旋转角度（单位：弧度）
        // static_transform.transform.rotation.x = q.x();
        // static_transform.transform.rotation.y = q.y();
        // static_transform.transform.rotation.z = q.z();
        // static_transform.transform.rotation.w = q.w();
        // static_broadcaster_->sendTransform(static_transform);

        // joint_link -> back_camera : 静态变换，后视相机相对转轴
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

        // 根据自我颜色设置初始位置
        if(!is_self_color_set_) return;
        geometry_msgs::msg::Transform transfrom_;
        if(!is_initial_pose_get_)  
        {
            transfrom_ = init_trans_;
        }
        else transfrom_ = initial_pose_guess_;

        // map -> odom : 初始化位置
        static_transform.header.frame_id = "map";   // 父坐标系
        static_transform.child_frame_id = "odom";    // 子坐标系
        static_transform.transform.translation.x = transfrom_.translation.x;
        static_transform.transform.translation.y = transfrom_.translation.y;
        static_transform.transform.translation.z = transfrom_.translation.z;
        static_transform.transform.rotation.x = transfrom_.rotation.x;
        static_transform.transform.rotation.y = transfrom_.rotation.y;
        static_transform.transform.rotation.z = transfrom_.rotation.z;
        static_transform.transform.rotation.w = transfrom_.rotation.w;
        static_broadcaster_->sendTransform(static_transform);

    }

    // 里程计回调
    void TfTransformer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        nav_msgs::msg::Odometry odom_transformed;
        geometry_msgs::msg::TransformStamped transform;
        geometry_msgs::msg::Pose transformedPose;
        geometry_msgs::msg::Pose rawPose;
        odom_transformed.twist = msg->twist;
        odom_transformed.header = msg->header;
        odom_transformed.pose.covariance = msg->pose.covariance;
        rawPose = msg->pose.pose;
        
        try {
            transform = tf_buffer_->lookupTransform("lidar_link", "base_footprint", rclcpp::Time(0), 0.1s);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s. Waiting for transform...", ex.what());
            return;  // 如果转换查找失败，则退出该回调
        }
        
        try {
            tf2::doTransform(rawPose, transformedPose, transform);
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


        geometry_msgs::msg::TransformStamped odom_transform;
        odom_transform.header.stamp = msg->header.stamp;
        odom_transform.header.frame_id = "odom";
        odom_transform.child_frame_id = "base_link";
        odom_transform.transform.translation.x = msg->pose.pose.position.x;
        odom_transform.transform.translation.y = msg->pose.pose.position.y;
        odom_transform.transform.translation.z = msg->pose.pose.position.z;
        odom_transform.transform.rotation.x = msg->pose.pose.orientation.x;
        odom_transform.transform.rotation.y = msg->pose.pose.orientation.y;
        odom_transform.transform.rotation.z = msg->pose.pose.orientation.z;
        odom_transform.transform.rotation.w = msg->pose.pose.orientation.w;
        tf_broadcaster_->sendTransform(odom_transform);
    }

    // 小陀螺回调（作用存疑）
    void TfTransformer::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg )
    {
        // try {
        //     // 获取 base_footprint_fake 到 base_footprint 的旋转关系
        //     auto transform = tf_buffer_->lookupTransform(
        //         "base_footprint", "base_footprint_fake", tf2::TimePointZero
        //     );

        //     // 转换速度向量
        //     geometry_msgs::msg::Twist transformed_vel;
        //     // tf2::doTransform(*msg, transformed_vel, transform);

        //     // 提取旋转矩阵
        //     tf2::Quaternion q;
        //     tf2::fromMsg(transform.transform.rotation, q);
        //     tf2::Matrix3x3 rotation_matrix(q);

        //     // 转换线速度
        //     tf2::Vector3 linear_vel(msg->linear.x, msg->linear.y, msg->linear.z);
        //     linear_vel = rotation_matrix * linear_vel;
        //     transformed_vel.linear.x = linear_vel.x();
        //     transformed_vel.linear.y = linear_vel.y();
        //     transformed_vel.linear.z = 0.0;
        //     transformed_vel.angular.x = 0.0;
        //     transformed_vel.angular.y = 0.0;
        //     transformed_vel.angular.z = 0.0;

        //     // 发布转换后的速度
        //     twist_pub_->publish(transformed_vel);
        // } catch (tf2::TransformException &ex) {
        //     RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
        //     return;
        // }

        // 暂时不使用 base_footprint_fake（避免 TF 警告刷屏）。
        // 直接将 /cmd_vel 转发到 /cmd_vel_fake。
        twist_pub_->publish(*msg);
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
