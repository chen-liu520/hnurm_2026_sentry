#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ground_segmentation/ground_segmentation.h"

class SegmentationNode : public rclcpp::Node {
public:
    SegmentationNode(const rclcpp::NodeOptions& node_options);
    void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // 发布器：分别发布地面和障碍物点云
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
    
    // TF变换：用于坐标系转换
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 订阅器：接收原始点云
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    
    // 分割算法参数和实例
    GroundSegmentationParams params_;
    std::shared_ptr<GroundSegmentation> segmenter_;
    
    // 重力对齐坐标系名称
    std::string gravity_aligned_frame_;
    
};

SegmentationNode::SegmentationNode(const rclcpp::NodeOptions& node_options)
    : Node("ground_segmentation", node_options) {
    // 坐标系配置
    gravity_aligned_frame_ = this->declare_parameter("gravity_aligned_frame", "gravity_aligned");

    // 可视化参数
    params_.visualize = this->declare_parameter("visualize", params_.visualize);

    // 分割结构参数
    params_.n_bins     = this->declare_parameter("n_bins", params_.n_bins);
    params_.n_segments = this->declare_parameter("n_segments", params_.n_segments);
    params_.n_threads  = this->declare_parameter("n_threads", params_.n_threads);

    // 直线拟合参数
    params_.max_dist_to_line  = this->declare_parameter("max_dist_to_line", params_.max_dist_to_line);
    params_.line_search_angle = this->declare_parameter("line_search_angle", params_.line_search_angle);

    // 斜率约束
    params_.max_slope = this->declare_parameter("max_slope", params_.max_slope);
    params_.min_slope = this->declare_parameter("min_slope", params_.min_slope);

    // 高度阈值
    params_.sensor_height    = this->declare_parameter("sensor_height", params_.sensor_height);
    params_.max_start_height = this->declare_parameter("max_start_height", params_.max_start_height);
    params_.max_long_height  = this->declare_parameter("max_long_height", params_.max_long_height);
    params_.long_threshold   = this->declare_parameter("long_threshold", params_.long_threshold);

    // 需要平方的参数
    double r_min = 0.5, r_max = 10.0, max_fit_error;
    if (this->get_parameter("r_min", r_min)) {
        params_.r_min_square = r_min * r_min;
    }
    if (this->get_parameter("r_max", r_max)) {
        params_.r_max_square = r_max * r_max;
    }
    if (this->get_parameter("max_fit_error", max_fit_error)) {
        params_.max_error_square = max_fit_error * max_fit_error;
    }

    // 创建地面分割器
    segmenter_ = std::make_shared<GroundSegmentation>(params_);

    // 话题配置
    std::string ground_topic   = this->declare_parameter("ground_output_topic", "ground_cloud");
    std::string obstacle_topic = this->declare_parameter("obstacle_output_topic", "obstacle_cloud");
    std::string input_topic    = this->declare_parameter("input_topic", "input_cloud");

    // 创建订阅者
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, rclcpp::SensorDataQoS(),
        std::bind(&SegmentationNode::scanCallback, this, std::placeholders::_1));

    // 创建发布者
    ground_pub_   = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        ground_topic, rclcpp::SensorDataQoS());
    obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        obstacle_topic, rclcpp::SensorDataQoS());

    // 初始化TF
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Segmentation node initialized");
}

void SegmentationNode::scanCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    // 将ROS点云消息转换为PCL点云格式
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
    pcl::fromROSMsg(*msg, cloud);

    // 坐标系转换
    bool is_original_pc = true;
    if (!gravity_aligned_frame_.empty()) {
        geometry_msgs::msg::TransformStamped tf_stamped;
        try {
            // 获取从点云坐标系到重力对齐坐标系的变换
            tf_stamped = tf_buffer_->lookupTransform(
                gravity_aligned_frame_, msg->header.frame_id, msg->header.stamp);
            // 移除平移部分，只保留旋转
            tf_stamped.transform.translation.x = 0;
            tf_stamped.transform.translation.y = 0;
            tf_stamped.transform.translation.z = 0;
            Eigen::Affine3d tf;
            tf.translate(Eigen::Vector3d(0, 0, 0));
            tf.rotate(Eigen::Quaterniond(
                tf_stamped.transform.rotation.w, tf_stamped.transform.rotation.x,
                tf_stamped.transform.rotation.y, tf_stamped.transform.rotation.z));
            // tf::transformMsgToEigen(tf_stamped.transform, tf);

            // 应用变换
            pcl::transformPointCloud(cloud, cloud_transformed, tf);
            is_original_pc = false;
        }
        catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(),
                "Failed to transform point cloud into "
                "gravity frame: %s",
                ex.what());
        }
    }

    // Trick to avoid PC copy if we do not transform.
    // 使用const引用避免不必要的点云复制
    // 如果没有进行坐标系变换，则直接使用原始点云
    const pcl::PointCloud<pcl::PointXYZ>& cloud_proc =
        is_original_pc ? cloud : cloud_transformed;

    // 进行地面分割
    std::vector<int> labels;
    segmenter_->segment(cloud_proc, &labels);

    // 点云分类
    pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (labels[i] == 1) // 地面点
            ground_cloud.push_back(cloud[i]);
        else                // 障碍物点
            obstacle_cloud.push_back(cloud[i]);
    }

    // 发布结果点云
    auto ground_msg   = std::make_shared<sensor_msgs::msg::PointCloud2>();
    auto obstacle_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    pcl::toROSMsg(ground_cloud, *ground_msg);
    pcl::toROSMsg(obstacle_cloud, *obstacle_msg);

    ground_msg->header   = msg->header;
    obstacle_msg->header = msg->header;

    ground_pub_->publish(*ground_msg);
    obstacle_pub_->publish(*obstacle_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<SegmentationNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
