#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "hnurm_interfaces/msg/vision_recv_data.hpp"
#include "hnurm_interfaces/msg/vision_send_data.hpp"
#include "hnurm_interfaces/msg/target.hpp"
#include <hnurm_interfaces/msg/zone_end_point2_d.hpp>
#include <hnurm_interfaces/msg/special_area.hpp>
#include <hnurm_interfaces/msg/area.hpp>
#include <hnurm_interfaces/msg/type.hpp>
#include <hnurm_interfaces/msg/self_color.hpp>
#include "std_msgs/msg/float32.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <Eigen/Dense>

#include <deque>
#include <mutex>
#include <atomic>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <cmath>
#include <stdexcept>

namespace hnurm_ul_behavior_trees
{

    class PubRobotStatus : public BT::SyncActionNode
    {
    public:
        PubRobotStatus(const std::string &xml_tag_name, const BT::NodeConfiguration &conf);
        ~PubRobotStatus() override;

        static BT::PortsList providedPorts()
        {
            // BT::OutputPort<bool>("黑板变量名", "黑板变量描述"),
            // setOutput("黑板变量名"， 变量值)
            return {
                // 状态机
                BT::OutputPort<bool>("pose_from_human", "pose_from_human_interrupt"), // 标记是否接受到云台手发送的目标点
                BT::OutputPort<bool>("need_supply", "need_supply"),                   // 标记是否需要补给
                BT::OutputPort<bool>("is_pursue", "is_pursue"),                       // 标记是否需要追击
                BT::OutputPort<bool>("is_cruise", "is_cruise"),                       // 标记是否需要巡航
                BT::OutputPort<bool>("is_in_upanddown_area", "is_in_upanddown_area"), // 标记是否在起伏路段区域内

                // 目标点
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("upanddown_goal", "upanddown area goal pose"),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("human_goal", "human goal pose"),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("supply_goal", "supply goal pose"),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("pursue_goal", "pursue goal pose"),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("cruise_goal", "cruise goal pose"),

                // 其他目标点队列

                // 其他
                BT::OutputPort<std::string>("current_controller", "current_controller"),
            };
        }

        BT::NodeStatus tick() override;

        rclcpp::Node::SharedPtr node_;

        rclcpp::CallbackGroup::SharedPtr callback_group_; // 常规回调组
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

        rclcpp::SubscriptionOptions sub_option;

        // 全局坐标点
        struct GlobalPose
        {
            float pose_x;
            float pose_y;
        };

        hnurm_interfaces::msg::VisionSendData send_target_information_;
        hnurm_interfaces::msg::VisionRecvData recv_robot_information_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        enum class SpecialAreaType
        {
            MY_HOME,
            MY_MOVE,
            CENTER,
            OPPONENT_HOME,
            OPPONENT_MOVE,

            ILLEGAL
        };
        enum class CruiseMode
        {
            INIT,          // 初始巡航路径
            CENTER,        // 中心点
            SUPPLY,        // 补给点
            AMBUSH,        // 伏击点
            GUARD,         // 防守点
            HOME           // 一直被限制在家里
        };

        enum class SelfColor
        {
            RED,
            BLUE,
        };

        enum class CenterOccupyStatus
        {
            NONE,
            MYSELF,
            OPPONENT,
            BOTH,
        };
        SelfColor self_color_; // 我方颜色

        CenterOccupyStatus center_occupy_status_; // 中心占领状态

        CenterOccupyStatus last_center_occupy_status_; // 上一次中心占领状态

        bool is_color_setted_ = false; // 是否已经从串口状态消息中获取到颜色信息

        bool is_init_ = false;

    private:
        /*****************************************/
        /*************1. 私有函数 start************/
        /*****************************************/

        // 1.1 订阅回调函数

        void global_pose_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

        // 重映射速度命令，实现特殊区域内的速度控制
        void remap_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

        //  接收视觉系统发送的机器人静态数据和状态信息
        void recv_callback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg);

        // 处理后视相机的目标数据（当前代码已注释）
        void back_target_callback(const std_msgs::msg::Float32::SharedPtr msg);

        // 接收视觉系统发送的目标状态信息，控制机器人行为模式切换
        void send_target_info_callback(const hnurm_interfaces::msg::VisionSendData::SharedPtr msg);

        // 巡航路径更新定时器回调函数
        void cruise_change_timer_callback();

        // 特殊区域内相关判断定时器回调函数
        void special_area_timer_callback();

        // 填充目标点队列定时器回调函数
        void fill_deque_timer_callback();

        // 1.2 工具函数
        // 1.2.1 工具最上层
        void CheckIsReached_Update(float threshold); // 检查巡航是否到达目标点，如果到达则更新目标点为队列中的下一个点

        void FillGoalsDeque_Cruise(); // 根据当前状态填充巡航点队列

        void UpdateCurrentCruiseMode(); // 根据当前状态和时间更新巡航模式

        void FillGoalsDeque_Pursue(); // 根据视觉系统发送的目标信息填充追击目标点队列

        SpecialAreaType GetPointArea(const GlobalPose &point); // 获取当前所在区域

        void ScanCenterAngleProcess(const double target_yaw_deg); // 处理扫描中心角度回调

        CenterOccupyStatus GetCenterOccupyStatus(float msg_num); // 获取当前占领状态

        // 1.2.2 工具函数细节实现，被最上层和回调调用
        geometry_msgs::msg::PoseStamped GlobalPose2PoseStamped(bool is_pursue);

        GlobalPose calculateAverage(geometry_msgs::msg::PolygonStamped &msg);

        float calculate_distance(const GlobalPose &pose1, const GlobalPose &pose2);

        void structure_get_params(); // 从参数服务器获取参数,填充相关变量，【被PubRobotStatus构造函数调用】

        void structure_get_cruise_paths_from_paramsandcolor(std::string color); // 从参数服务器获取巡航路径点,填充path_pose_arrays_，【被PubRobotStatus构造函数调用】

        void structure_ROS_init(); // 初始化ROS节点、发布者、订阅者，【被PubRobotStatus构造函数调用】

        void initialization(); // 其他初始化工作，【被PubRobotStatus构造函数调用】

        geometry_msgs::msg::Pose get_targetpose_in_map(float &distance);

        void create_pursue_stay_circle_path(); // 创建追击静止小陀螺的路径点队列，围绕目标点做一个半径为0.5m的圆

        bool is_in_this_polygon(const GlobalPose &point, const std::vector<GlobalPose> &polygon);

        CruiseMode stringToCruiseMode(const std::string &mode_str);

        std::string cruiseModeToString(CruiseMode mode);

        std::string specialAreaToString(SpecialAreaType area);

        /*******************************************/
        /***************1. 私有函数 end **************/
        /*******************************************/

        /**************************2. 订阅、发布相关变量 start******************************/
        // 订阅全局【定位】系统发布的机器人位置
        rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr global_pose_sub_;

        // 订阅速度命令,重映射
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

        // 订阅串口发上来的状态信息
        rclcpp::Subscription<hnurm_interfaces::msg::VisionRecvData>::SharedPtr recv_sub_;

        // 订阅视觉发布的目标信息，用于切换追击和巡航状态，以及获取目标距离，目标类型
        rclcpp::Subscription<hnurm_interfaces::msg::VisionSendData>::SharedPtr send_target_info_sub_;

        // 后视相机目标信息订阅
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr back_target_sub_;
        
        // 发布姿态(小)
        rclcpp::Publisher<hnurm_interfaces::msg::Gesture>::SharedPtr gesture_pub_;

        // 发布速度命令,重映射
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_remap_pub_;

        // 控制小陀螺旋转模式
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr spin_control_pub_;

        // 控制180度扫描中心角度
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr scan_center_angle_pub_;

        // 发布目标是否在敌方基地
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_target_at_home_pub_;

        // 发布是否启用180度扫描
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_180_scan_pub_;

        // ROS2定时器
        rclcpp::TimerBase::SharedPtr cruise_change_timer_; // 定时检查是否需要切换巡航路径的定时器

        rclcpp::TimerBase::SharedPtr special_area_timer_; // 特殊区域内相关判断定时器

        rclcpp::TimerBase::SharedPtr fill_deque_timer_; // 填充目标点队列定时器,高频
        // topic名称参数

        std::string cmd_vel_topic_;
        std::string global_position_topic_;
        std::string cmd_vel_remap_topic_;
        std::string recv_topic_;
        std::string send_topic_;
        std::string back_target_topic_;
        std::string spin_control_topic_; // 发布进入起伏路段消息的话题 
        std::string scan_center_angle_topic_; // 发布控制180度扫描中心角度的话题
        std::string back_target_state_pub_topic_; // 发布回防目标状态的话题
        std::string gesture_pub_topic_; // 发布姿态(小)的话题
        std::string enable_180_scan_topic_; // 发布是否启用180度扫描的话题
        std::string is_target_at_home_topic_; // 发布目标是否在我方基地的话题

        /**************************2. 订阅、发布相关变量 end******************************/

        /*********************3. 巡航相关变量 start ******************************/
        // 读取参数文件，获取巡航、补给、特殊点位、特殊区域、特殊路径的所有路径点，通过名字获取
        
        std::unordered_map<CruiseMode, std::vector<GlobalPose>> path_pose_arrays_;

        // 当前的目标点队列
        std::deque<GlobalPose> current_cruise_goal_deque_;

        GlobalPose current_x_y_; // 当前机器人的坐标点,来自nav2的footprint回调

        CruiseMode current_cruise_mode_;  // 当前巡航模式
        CruiseMode previous_cruise_mode_; // 记录上一次巡航模式，用于判断巡航模式是否切换

        rclcpp::Time current_cruise_mode_start_time_; // 当前巡航模式开始的时间点

        rclcpp::Duration change_cruise_mode_time_threshold_ = rclcpp::Duration::from_seconds(20); // 切换巡航模式的时间阈值，超过这个时间还没有切换则强制切换
        /*********************3. 巡航相关变量 end  ******************************/

        /************************4.  追击相关变量 start ******************************/
        // 追击探测到目标后将current_cruise_mode_start_time重置为当前时间，主要防止单巡航路线长时间运行但是没有起作用
        enum class PursueMode
        {
            PURSUE, // 正常追击，发送对方位置
            STAY,
            STAY_CIRCLE,
            FALLBACK
        };

        PursueMode current_pursue_mode_; // 当前追击模式

        std::deque<GlobalPose> pursue_goal_deque_; // 追击目标点队列

        float pursue_stay_circle_radius_; // 追击静止小陀螺的半径

        SpecialAreaType current_target_area_; // 当前目标点所在区域
        /************************4. 追击相关变量 end ******************************/

        /************************5. 特殊区域相关变量 start ******************************/
        std::unordered_map<SpecialAreaType, std::vector<GlobalPose>> special_area_poses_;   // 特殊区域的所有边界点，通过名字获取
        std::unordered_map<std::string, std::vector<GlobalPose>> special_area_from_params_; // 从参数文件中读取的特殊区域的所有边界点，通过名字获取

        SpecialAreaType current_my_area_; // 当前所在区域
        /************************5. 特殊区域相关变量 end ******************************/

        /************************6. FSM状态相关变量 start ******************************/
        bool is_pose_from_human_; // 标记是否接受到云台手发送的目标点

        bool is_need_supply_; // 标记是否需要补给

        bool is_pursue_; // 标记是否需要追击

        bool is_cruise_; // 标记是否需要巡航
        /************************6. FSM状态相关变量 end ******************************/


        /**************************8. 线程和锁 start ******************************/


        std::thread executor_thread_;               // 执行 callback_group_executor_ 的线程
        std::atomic<bool> executor_running_{false}; // executor 运行标志

        std::mutex fsm_state_mutex_;
        /**************************8. 线程和锁 end ******************************/

        /***************************9. 重映射速度控制 *****************************/
        // geometry_msgs::msg::Twist last_cmd_vel_; // 上一次发布的速度
        // bool has_last_cmd_vel_ = false;          // 是否有上一次速度记录
        // const double MAX_LINEAR_ACCEL = 2.0;     // 最大线加速度 (m/s²)
        // const double DT = 0.01;                  // 控制周期 10ms (100Hz)
        /***************************9. 重映射速度控制 *****************************/

        /***************************10. 后视相机相关变量 start ******************************/
        bool is_back_target_detected_; // 是否检测到后视相机目标
        int back_count_ = 0;           // 控制频率，限定时间内不再触发
        /***************************10. 后视相机相关变量 end ******************************/

        /***************************11. 其他相关变量 start ******************************/
        std::chrono::steady_clock::time_point last_recv_time_; // 上次recv回调运行时间点，用于控制频率

        std_msgs::msg::Float32 last_spin_control_msg_; // 上次发布进入起伏路段消息的时间点，用于控制频率
        /***************************11. 其他相关变量 end ******************************/
    };
}
