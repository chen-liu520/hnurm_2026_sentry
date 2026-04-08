#include "pointcloud_filter/pointcloud_filter_node.hpp"
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>


namespace hnurm {

    PointCloudNode::PointCloudNode(const rclcpp::NodeOptions& options)
        : Node("PointCloudNode", options)
    {
        RCLCPP_INFO(get_logger(), "PointCloudNode is running");

        // 声明变量
        lidar_topic_        = this->declare_parameter("lidar_topic", "segmentation/obstacle");     //segmentation/obstacle    /livox/lidar/pointcloud
        odom_topic_         = this->declare_parameter("odom_topic", "/Odometry");
        sensor_height_      = this->declare_parameter("sensor_height", 0.31);
        output_lidar_topic_ = this->declare_parameter("output_lidar_topic_", "/pointcloud");
        output_laser_topic_ = this->declare_parameter("output_laser_topic_", "/ld_laserscan");
        base_frame_         = this->declare_parameter("base_frame", "base_footprint");
        lidar_frame_        = this->declare_parameter("lidar_frame", "lidar_link");
        laser_topic_        = this->declare_parameter("laser_topic", "/laser/scan");
        laser_frame_        = this->declare_parameter("laser_frame", "laser_link");
        robot_radius_       = this->declare_parameter("robot_radius", 0.5);
        filter_topic_       = this->declare_parameter("filter_topic", "/special_areas");

        // 初始化TF监听器
        tf_buffer_          = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_        = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);


        // ============================
        // 创建订阅者和发布者
        // ============================

        // 定义可靠传输的 QoS 配置（用于激光扫描发布）
        rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
        qos_profile.reliable();  // 明确设置为可靠传输，确保消息不丢失

        // ------------------------
        // 订阅者（Subscribers）
        // ------------------------

        // 订阅输入点云数据（已去畸变或注册后的点云）
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PointCloudNode::pointcloud_callback, this, std::placeholders::_1)
        );

        // 订阅输入激光扫描数据
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PointCloudNode::laser_callback, this, std::placeholders::_1)
        );

        // 订阅里程计数据（用于获取当前传感器高度）
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PointCloudNode::odom_callback, this, std::placeholders::_1)
        );

        // 订阅特殊区域消息（用于区域过滤）
        spa_sub_ = this->create_subscription<hnurm_interfaces::msg::SpecialArea>(
            filter_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PointCloudNode::spa_callback, this, std::placeholders::_1)
        );

        // 订阅是否在特殊区域的标志（已弃用，保留接口兼容性）
        in_special_area_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/is_in_special_area",
            rclcpp::SensorDataQoS(),
            std::bind(&PointCloudNode::is_in_SpecialArea_callback, this, std::placeholders::_1)
        );

        // ------------------------
        // 发布者（Publishers）
        // ------------------------

        // 发布过滤后的点云数据
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_lidar_topic_,
            rclcpp::SensorDataQoS()
        );

        // 发布过滤后的激光扫描数据（使用可靠传输）
        laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            output_laser_topic_,
            qos_profile
        );

        // 发布测试用激光数据（调试用）
        test_laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/test_laser",
            qos_profile
        );

        // 发布变换后的特殊区域（已转换到传感器坐标系，供调试或下游节点使用）
        tfed_spa_pub_ = this->create_publisher<hnurm_interfaces::msg::SpecialArea>(
            "/transformed_special_area",
            rclcpp::SensorDataQoS()
        );

    }

    void PointCloudNode::is_in_SpecialArea_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // use_filter = msg->data;     //old method,do not use
    }

    void PointCloudNode::spa_callback(const hnurm_interfaces::msg::SpecialArea::SharedPtr msg)
    {
        current_spa_ = *msg;
        use_filter = true;  //catch the special area ,turn the filter spa mode on
    }

    void PointCloudNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_sensor_height = msg->pose.pose.position.z;
    }

    // 转换特殊地块，从map坐标系转换到传感器坐标系
    hnurm_interfaces::msg::SpecialArea PointCloudNode::transform_SpecialArea(const geometry_msgs::msg::TransformStamped& transform, const hnurm_interfaces::msg::SpecialArea& spa)
    {

        hnurm_interfaces::msg::SpecialArea transformed_spa_ = spa;
        transformed_spa_.points.clear();  //clear pre points

        Eigen::Isometry3d transform_matrix = Eigen::Isometry3d::Identity();
        //translation matrix
        transform_matrix.translation() = Eigen::Vector3d(
            transform.transform.translation.x,
            transform.transform.translation.y,
            0.0
            // transform.transform.translation.z
        );

        // rotation matrix
        Eigen::Quaterniond rotation(
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z
        );
        transform_matrix.rotate(rotation.normalized());


        //SpecialArea.points type : std::vector<ZoneEndPoint2D> polygon is closed
        for (auto sp : spa.points)
        {
            Eigen::Vector3d point_3d(sp.x, sp.y, 0.0);
            Eigen::Vector3d transformed_3d = transform_matrix * point_3d;
            hnurm_interfaces::msg::ZoneEndPoint2D transformed_point;
            transformed_point.x = transformed_3d.x();
            transformed_point.y = transformed_3d.y();
            transformed_spa_.points.push_back(transformed_point);
        }
        return transformed_spa_;
    }

    // bool PointCloudNode::is_in_SpecialArea(float x,float y,const hnurm_interfaces::msg::SpecialArea& area)
    // {
    //   bool inside = false;
    //     hnurm_interfaces::msg::SpecialArea temp_spa_ = area;
    //     const size_t n = temp_spa_.points.size();

    //     for(size_t i = 0, j = n-1; i < n; j = i++) {
    //         const double xi = temp_spa_.points[i].x, yi = temp_spa_.points[i].y;
    //         const double xj = temp_spa_.points[j].x, yj = temp_spa_.points[j].y;

    //         const bool intersect = ((yi > x) != (yj > y)) &&
    //             (x < (xj - xi) * (y - yi) / (yj - yi) + xi);

    //         if(intersect) {
    //             inside = !inside;
    //         }
    //     }
    //     return inside;
    // }

    // 点(x, y)是否在多边形area内，使用射线法
    bool PointCloudNode::is_in_SpecialArea(float x, float y, const hnurm_interfaces::msg::SpecialArea& area) {
        bool inside = false;
        const size_t n = area.points.size();
        if (n < 3) return false;
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            const float xi = area.points[i].x, yi = area.points[i].y;
            const float xj = area.points[j].x, yj = area.points[j].y;
            const bool y_in_range = (yi > y) != (yj > y);
            if (y_in_range) {
                const float dx = xj - xi, dy = yj - yi;
                if (dy == 0) continue; // avoid divide zero
                const float t = (y - yi) / dy;
                const float x_intersect = xi + t * dx;
                if (x <= x_intersect) {
                    inside = !inside;
                }
            }
        }
        return inside;
    }

    // 激光雷达数据回调函数
    void PointCloudNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        hnurm_interfaces::msg::SpecialArea temp_tfed_spa_;
        if (use_filter)
        {
            // // 1. obtain transform map->laser link
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer_->lookupTransform(
                    // target frame current_spa_(map) -> temp_tfed_spa_(laser_link or lidar_link))
                    msg->header.frame_id,  // pointcloud frame
                    "map",
                    rclcpp::Time(0),       // 使用最新可用变换
                    rclcpp::Duration::from_seconds(0.1)
                );
            }
            catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "tf failed: %s", ex.what());
                return;
            }
            temp_tfed_spa_ = transform_SpecialArea(transform, current_spa_);
        }
        // filtering
        auto filtered_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
        //debug test 
        sensor_msgs::msg::LaserScan test_;
        // test_.header = msg->header;

        const size_t num_points = msg->ranges.size();
        filtered_scan->ranges.resize(msg->ranges.size());

        //angle pre-calculation
        std::vector<float> angles(num_points);
        for (size_t i = 0; i < num_points; ++i) {
            angles[i] = msg->angle_min + i * msg->angle_increment;
        }

        // #pragma omp parallel for
        for (size_t i = 0; i < num_points; ++i) {
            const float range = msg->ranges[i];
            if (!std::isfinite(range) || range > msg->range_max) {
                filtered_scan->ranges[i] = range;
                continue;
            }

            // calculate current point in the laser_link
            const float x = range * std::cos(angles[i]);
            const float y = range * std::sin(angles[i]);

            // 计算到车体中心的水平距离
            const float distance = std::hypot(x, y);
            //filter datas in special area 

            if (use_filter)
            {
                if (is_in_SpecialArea(x, y, temp_tfed_spa_) || distance <= robot_radius_)
                {
                    // test_.ranges.push_back(filtered_scan->ranges[i]);
                    filtered_scan->ranges[i] = std::numeric_limits<float>::infinity(); // remove
                }
                else filtered_scan->ranges[i] = range;  // keep
            }
            else if (distance <= robot_radius_) {
                filtered_scan->ranges[i] = std::numeric_limits<float>::infinity(); // remove
            }
            else filtered_scan->ranges[i] = range;  // keep
        }
        // 4. 发布过滤后的数据
        filtered_scan->header.frame_id = laser_frame_;  // 更新坐标系
        laser_pub_->publish(*filtered_scan);
        // test_laser_pub_->publish(test_);
    }


    // sensor_msgs::msg::PointCloud2 PointCloudNode::create_empty_pointcloud(const sensor_msgs::msg::PointCloud2& msg )
    // {
    //     sensor_msgs::msg::PointCloud2 cloud;
    //     cloud.header =msg.heade;
    //     // cloud.data.clear();
    //     return cloud;
    // }

    // 点云数据回调函数
    void PointCloudNode::pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 1. obtain transform map->lidar link
        hnurm_interfaces::msg::SpecialArea temp_tfed_spa_;
        if (use_filter)
        {
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer_->lookupTransform(
                    // target frame current_spa_(map) -> temp_tfed_spa_(laser_link or lidar_link))
                    msg->header.frame_id,  // pointcloud frame
                    "map",
                    rclcpp::Time(0),       // 使用最新可用变换
                    rclcpp::Duration::from_seconds(0.1)
                );
            }
            catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "tf failed: %s", ex.what());
                return;
            }
            temp_tfed_spa_ = transform_SpecialArea(transform, current_spa_);
            tfed_spa_pub_->publish(temp_tfed_spa_);
        }
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        // // 构建变换矩阵（先旋转后平移）
        // Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
        // transform_matrix.translate(Eigen::Vector3f(
        //     transform.transform.translation.x,
        //     transform.transform.translation.y,
        //     0.0
        // ));
        // transform_matrix.rotate(Eigen::Quaternionf(
        //     transform.transform.rotation.w,
        //     transform.transform.rotation.x,
        //     transform.transform.rotation.y,
        //     transform.transform.rotation.z
        // ));

        // pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        // pcl::transformPointCloud(cloud, transformed_cloud, transform_matrix);   //取消变换

        //过滤车体半径内的点
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
        // for (const auto &point : transformed_cloud) {
        for (const auto& point : cloud) {
            const float distance = std::hypot(point.x, point.y);
            if (use_filter)
            {
                if (is_in_SpecialArea(point.x, point.y, temp_tfed_spa_) || distance < robot_radius_) {
                    continue;
                }
                else filtered_cloud.push_back(point);
            }
            else if (distance > robot_radius_) {
                // if (point.z<sensor_height_/2.0)
                // {
                //   continue;
                // }
                // else filtered_cloud.push_back(point);  //似乎没啥效果，先不做
                filtered_cloud.push_back(point);
            }
        }

        //发布过滤后的点云
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(filtered_cloud, filtered_msg);
        filtered_msg.header.stamp = msg->header.stamp;
        // filtered_msg.header.frame_id = base_frame_;  // 坐标系已转换到车体
        filtered_msg.header.frame_id = lidar_frame_;  // 坐标系已转换到车体
        pointcloud_pub_->publish(filtered_msg);
    }

}