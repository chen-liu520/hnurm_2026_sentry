#include "pointcloud_filter/pointcloud_filter_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    // options.allow_undeclared_parameters(true)
    //         .automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<hnurm::PointCloudNode>(options);
    // rclcpp::spin(node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}