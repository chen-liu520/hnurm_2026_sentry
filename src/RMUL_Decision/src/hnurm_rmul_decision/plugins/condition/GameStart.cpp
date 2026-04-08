#include "GameStart.hpp"
#include <termios.h>
#include <unistd.h>
#include <iostream>

namespace hnurm_ul_behavior_trees
{

    GameStart::GameStart(const std::string &condition_name, const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          game_progress_(0.0f), // 默认初始为0，避免意外
          threshold_(3.5f)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        RCLCPP_INFO(node_->get_logger(), "[GameStart] 初始化完成，等待game_progress > %.1f...", threshold_);

        game_start_time_ = node_->now();

        // 创建订阅，监听 /vision_recv_data
        // 与 uart 发布端保持一致，避免 RELIABILITY_QOS_POLICY 不兼容
        game_start_sub_ = node_->create_subscription<hnurm_interfaces::msg::VisionRecvData>(
            "/vision_recv_data",
            rclcpp::SensorDataQoS(),
            std::bind(&GameStart::gameStartCallback, this, std::placeholders::_1));

    }

    GameStart::~GameStart()
    {
    }

    BT::NodeStatus GameStart::tick()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        
        return BT::NodeStatus::SUCCESS;

    }

    // BT::NodeStatus GameStart::tick()
    // {
    //     std::lock_guard<std::mutex> lock(mutex_);
    //     if(game_progress_ >= threshold_){
    //         return BT::NodeStatus::SUCCESS;
    //     }
    //     current_time_ = node_->now();
    //     rclcpp::Duration now_diff_time = current_time_ - game_start_time_;

    //     RCLCPP_INFO(node_->get_logger(), "\033[36m[GameStart::tick] game_progress: %.2f, time since start: %.2f seconds\033[0m",
    //                 game_progress_, now_diff_time.seconds());
    //     if(now_diff_time > rclcpp::Duration::from_seconds(200.0)){
    //         return BT::NodeStatus::SUCCESS;
    //     }
        
    //     return BT::NodeStatus::FAILURE;
            
        

    // }


    void GameStart::gameStartCallback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        game_progress_ = msg->game_progress;
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<hnurm_ul_behavior_trees::GameStart>("GameStart");
}