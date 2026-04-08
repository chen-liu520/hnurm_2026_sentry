//
// Created by rm on 24-4-8.
//

/// This node is implementing a crossing fire for two-head sentry
// 双头哨兵（废弃方案）

#include "hnurm_bringup/sentry_manage_node.h"
#include <angles/angles.h>

namespace hnurm
{
SentryManageNode::SentryManageNode(const rclcpp::NodeOptions &options) : rclcpp::Node("SentryManageNode", options)
{
    init_params();

    master_res_sub_ = this->create_subscription<hnurm_interfaces::msg::VisionSendData>(
        master_res_sub_topic_, 10, std::bind(&SentryManageNode::master_res_callback, this, std::placeholders::_1)
    );
    slave_res_sub_ = this->create_subscription<hnurm_interfaces::msg::VisionSendData>(
        slave_res_sub_topic_, 10, std::bind(&SentryManageNode::slave_res_callback, this, std::placeholders::_1)
    );

    master_res_pub_ = this->create_publisher<hnurm_interfaces::msg::VisionSendData>(master_res_pub_topic_, 10);
    slave_res_pub_  = this->create_publisher<hnurm_interfaces::msg::VisionSendData>(slave_res_pub_topic_, 10);
}

void SentryManageNode::init_params()
{
    master_res_sub_topic_ = this->declare_parameter("master_res", "/left/vision_send_res");
    slave_res_sub_topic_  = this->declare_parameter("slave_res", "/right/vision_send_res");

    master_res_pub_topic_ = this->declare_parameter("master_send_res", "/left/vision_send_data");
    slave_res_pub_topic_  = this->declare_parameter("slave_send_res", "/right/vision_send_data");

    use_cross_fire         = this->declare_parameter("use_cross_fire", true);
    master_to_slave_offset = this->declare_parameter("master_to_slave_offset", 0.335);

    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "master_res_sub_topic: " << master_res_sub_topic_ << ", slave_res_sub_topic: " << slave_res_sub_topic_
    );
}

void SentryManageNode::master_res_callback(hnurm_interfaces::msg::VisionSendData::ConstSharedPtr msg)
{
    if(use_cross_fire)
    {
        if(msg->target_state.data == hnurm_interfaces::msg::TargetState::FIRE)
            last_master_res_ = msg;
    }
    master_res_pub_->publish(*msg);
}

void SentryManageNode::slave_res_callback(hnurm_interfaces::msg::VisionSendData::ConstSharedPtr msg)
{
    if(!use_cross_fire)
    {
        slave_res_pub_->publish(*msg);
        return;
    }

    if(msg->target_state.data == hnurm_interfaces::msg::TargetState::FIRE)
    {
        // this means slave head has target
        slave_res_pub_->publish(*msg);
        return;
    }

    if(msg->target_state.data == hnurm_interfaces::msg::TargetState::CONVERGING)
    {
        // this means slave head has target
        slave_res_pub_->publish(*msg);
        return;
    }

    if(!last_master_res_)
    {
        // no target in master
        slave_res_pub_->publish(*msg);
        return;
    }

    // cross fire
    RCLCPP_INFO(this->get_logger(), "crossing fire");

    //             ▲                        ▲
    //             │                        │
    //             │                        │
    //     ┌─┐     │          ┌─┐           │     ┌─┐
    //     │0│     │          │0│           │     │0│           0: theta \in [-pi/2, pi/2] = theta*
    //     └─┘     │          └─┘           │     └─┘
    //             │                        │                   1: theta \in [-3pi/2, -pi] = theta* - pi
    //             │           d            │
    // ────────────x────────────────────────x─────────────      2: theta \in [-pi, -pi/2 ] = theta* - pi/2
    //             │Master                  │Slave
    //             │                        │
    //     ┌─┐     │          ┌─┐           │     ┌─┐                          d+r*sin(phi)
    //     │1│     │          │1│           │     │2│          theta* = arctan ───────────── \in [-pi/2,pi/2]
    //     └─┘     │          └─┘           │     └─┘                           r*cos(phi)
    //             │                        │
    //             │                        │

    // get target
    const auto &d   = master_to_slave_offset;
    const auto &r   = last_master_res_->target_distance;
    const auto  phi = angles::from_degrees(last_master_res_->yaw);
    // theta is target angle
    const auto theta      = std::atan((d + r * std::sin(phi)) / (r * std::cos(phi)));
    auto       yaw_target = angles::to_degrees(theta);

    if(phi > -M_PI_2 && phi < M_PI_2)
    {
        // 0
    }
    else if(phi > M_PI_2)  // may be 1 or 2
    {
        if(d + r * sin(phi) > 0)  // 1
            yaw_target = yaw_target - 180;
        else  // 2
            yaw_target = yaw_target - 90;
    }

    // clamp for yaw_target
    yaw_target = std::clamp(yaw_target, -215.0, 50.0);  // should be tested

    {
        hnurm_interfaces::msg::VisionSendData res;
        res.header            = msg->header;
        res.target_state.data = hnurm_interfaces::msg::TargetState::CONVERGING;
        res.target_type       = last_master_res_->target_type;
        res.pitch             = last_master_res_->pitch;
        res.yaw               = static_cast<float>(yaw_target);
        res.control_id        = 1.000;
        slave_res_pub_->publish(res);
        // reset
        last_master_res_.reset();
    }
}
}  // namespace hnurm

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<hnurm::SentryManageNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}