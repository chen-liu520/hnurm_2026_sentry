#include "PubRobotStatus.hpp"
#include <string>
#include <chrono>
#include "hnurm_interfaces/msg/gesture.hpp"
#include "behaviortree_cpp_v3/basic_types.h"

/*注释颜色：初始化：绿色，正常的：白色，错误的：红色，警告的：黄色*/
/*定时器监控所有状态：紫色35m*/
/*tick函数：蓝色34m*/
/*所有回调和调用的工具函数：青色36m*/
namespace hnurm_ul_behavior_trees
{
    using std::placeholders::_1;

    PubRobotStatus::PubRobotStatus(
        const std::string &xml_tag_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(xml_tag_name, conf)
    {
        node_ = conf.blackboard->get<rclcpp::Node::SharedPtr>("node");
        /**************************回调组 start**************************************/
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        sub_option.callback_group = callback_group_;

        /**************************回调组 end**************************************/

        structure_get_params(); // 从参数服务器获取参数,填充相关变量

        structure_ROS_init();

        executor_running_ = true;
        executor_thread_ = std::thread([this]()
                                       {
            RCLCPP_INFO(node_->get_logger(), "[PubRobotStatus] callback_group_executor_ 线程启动");
            while (executor_running_) {
                callback_group_executor_.spin_some(std::chrono::milliseconds(10));
            }
            RCLCPP_INFO(node_->get_logger(), "[PubRobotStatus] callback_group_executor_ 线程退出"); });


        std_msgs::msg::Float32 spin_control_msg_;
        spin_control_msg_.data = 4.0; // 小陀螺
        spin_control_pub_->publish(spin_control_msg_);
        last_spin_control_msg_ = spin_control_msg_;
        RCLCPP_INFO(node_->get_logger(), "PubRobotStatus::构造函数完成");
    }

    PubRobotStatus::~PubRobotStatus()
    {
        if (executor_thread_.joinable())
        {
            executor_thread_.join(); // 这个析构函数的主要作用是确保线程资源的正确回收
        }
        // 如果不调用 join()，线程可能在对象销毁后继续运行，导致以下问题：
        // 访问已释放的对象内存，引发未定义行为
        // 线程资源（如栈空间）无法被操作系统回收，造成内存泄漏
    }
 
    //  模式、状态等相关变量初始化
    void PubRobotStatus::initialization()
    {

        previous_cruise_mode_ = CruiseMode::SUPPLY; // 初始化为默认巡航模式
        current_cruise_mode_ = CruiseMode::INIT;    // 初始化为默认巡航模式
        current_cruise_mode_start_time_ = node_->now(); // 初始化时间戳，确保时间源一致
        current_pursue_mode_ = PursueMode::STAY;          // 初始化为默认追击模式
        is_cruise_ = true;                          // 默认初始为巡航模式
        is_need_supply_ = false;                    // 默认不需要补给
        is_pursue_ = false;                         // 默认不需要追击
        last_spin_control_msg_.data = 3.0;

        if (self_color_ == SelfColor::RED)
        {
            structure_get_cruise_paths_from_paramsandcolor("red");
            special_area_poses_[SpecialAreaType::MY_HOME] = special_area_from_params_["my_home_polygon"];
            special_area_poses_[SpecialAreaType::MY_MOVE] = special_area_from_params_["my_move_polygon"];
            special_area_poses_[SpecialAreaType::CENTER] = special_area_from_params_["center_polygon"];
            special_area_poses_[SpecialAreaType::OPPONENT_HOME] = special_area_from_params_["opponent_home_polygon"];
            special_area_poses_[SpecialAreaType::OPPONENT_MOVE] = special_area_from_params_["opponent_move_polygon"];
        }
        else
        {
            structure_get_cruise_paths_from_paramsandcolor("blue");
            special_area_poses_[SpecialAreaType::MY_HOME] = special_area_from_params_["my_home_polygon"];
            special_area_poses_[SpecialAreaType::MY_MOVE] = special_area_from_params_["my_move_polygon"];
            special_area_poses_[SpecialAreaType::CENTER] = special_area_from_params_["center_polygon"];
            special_area_poses_[SpecialAreaType::OPPONENT_HOME] = special_area_from_params_["opponent_home_polygon"];
            special_area_poses_[SpecialAreaType::OPPONENT_MOVE] = special_area_from_params_["opponent_move_polygon"];
        }

        is_init_ = true; // 初始化完成
    }

    void PubRobotStatus::structure_get_cruise_paths_from_paramsandcolor(std::string color)
    {
        const std::string GREEN = "\033[32m";
        const std::string RESET = "\033[0m";
        std::vector<std::string> cruise_goals_names = {
            "initial_goals", "supply_goals", "center_goals",
            "ambush_goals", "guard_goals", "home_goals"};
        for (const auto &name : cruise_goals_names)
        {
            std::string param_name = color + "_goals_array." + name;
            node_->declare_parameter(param_name, std::vector<std::string>());
            auto goals_str = node_->get_parameter_or(param_name, std::vector<std::string>());

            if (!goals_str.empty())
            {
                std::vector<GlobalPose> goals;
                for (const auto &point_str : goals_str)
                {
                    GlobalPose gp;
                    size_t comma_pos = point_str.find(',');
                    if (comma_pos != std::string::npos)
                    {
                        gp.pose_x = std::stof(point_str.substr(0, comma_pos));
                        gp.pose_y = std::stof(point_str.substr(comma_pos + 1));
                        goals.push_back(gp);
                    }
                }
                path_pose_arrays_[stringToCruiseMode(name)] = goals;
                RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params]加载 %s: %zu 个目标点%s",
                            GREEN.c_str(), name.c_str(), goals.size(), RESET.c_str());
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "%s[structure_get_params] %s is empty or not set%s",
                             GREEN.c_str(), name.c_str(), RESET.c_str());
            }
        }
    }

    /* @name: structure_ROS_init
     *  @brief：话题订阅，发布的初始化
     *  @param：
     */
    void PubRobotStatus::structure_ROS_init()
    {
        // 1. nav2的footprint回调订阅，获取当前机器人坐标点
        global_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
            global_position_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PubRobotStatus::global_pose_callback, this, _1),
            sub_option);

        // 2.速度订阅
        cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PubRobotStatus::remap_cmd_vel_callback, this, _1),
            sub_option);

        // 3. 订阅串口状态
        recv_sub_ = node_->create_subscription<hnurm_interfaces::msg::VisionRecvData>(
            recv_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PubRobotStatus::recv_callback, this, _1),
            sub_option);

        // 4. 发布串口状态订阅，【更新目标信息】
        send_target_info_sub_ = node_->create_subscription<hnurm_interfaces::msg::VisionSendData>(
            send_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PubRobotStatus::send_target_info_callback, this, _1),
            sub_option);

        // 5. 后视相机目标信息订阅
        back_target_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
            back_target_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PubRobotStatus::back_target_callback, this, _1),
            sub_option);

        // x.速度发布
        cmd_vel_remap_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_remap_topic_, 10);

        // 发布小陀螺控制
        spin_control_pub_ = node_->create_publisher<std_msgs::msg::Float32>(spin_control_topic_, 10);

        // 发布控制180度扫描中心角度
        scan_center_angle_pub_ = node_->create_publisher<std_msgs::msg::Float32>(scan_center_angle_topic_, 10);

        // 发布目标是否在敌方基地
        is_target_at_home_pub_ = node_->create_publisher<std_msgs::msg::Bool>(is_target_at_home_topic_, 10);

        // 发布是否启用180度扫描
        enable_180_scan_pub_ = node_->create_publisher<std_msgs::msg::Bool>(enable_180_scan_topic_, 10);

        // x.姿态发布(小)
        gesture_pub_ = node_->create_publisher<hnurm_interfaces::msg::Gesture>(gesture_pub_topic_, 10);

        // x. 发布巡航路径切换定时器
        cruise_change_timer_ = node_->create_wall_timer(std::chrono::seconds(4), std::bind(&PubRobotStatus::cruise_change_timer_callback, this), callback_group_);

        // x. 区域判断等
        special_area_timer_ = node_->create_wall_timer(std::chrono::milliseconds(223), std::bind(&PubRobotStatus::special_area_timer_callback, this), callback_group_);

        fill_deque_timer_ = node_->create_wall_timer(std::chrono::milliseconds(4), std::bind(&PubRobotStatus::fill_deque_timer_callback, this), callback_group_);
        
        // xx.初始化tf2_ros::Buffer和tf2_ros::TransformListener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    /*  @name : structure_get_params
     *  @brief :读取参数文件，获取巡航、补给、特殊点位、特殊区域、特殊路径的所有路径点，通过名字获取
     *  @param :
     *  @return :
     */
    void PubRobotStatus::structure_get_params()
    {
        // 绿色ANSI颜色代码
        const std::string GREEN = "\033[32m";
        const std::string RESET = "\033[0m";

        // 0. 声明参数
        node_->declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));
        node_->declare_parameter("global_position_topic", std::string("/global_costmap/published_footprint"));
        node_->declare_parameter("cmd_vel_remap_topic", std::string("/cmd_vel_remap"));
        node_->declare_parameter("recv_topic", std::string("/vision_recv_data"));
        node_->declare_parameter("send_topic", std::string("/vision_send_data"));
        node_->declare_parameter("back_target_topic", std::string("/back_target"));
        node_->declare_parameter("relocalization_status_topic", std::string("/registration_status"));
        node_->declare_parameter("pursue_stay_circle_radius", 0.5); // 追击静止小陀螺的半径，默认0.5米
        node_->declare_parameter("spin_control_pub_topic", std::string("/decision/spin_control"));
        node_->declare_parameter("scan_center_angle_topic", std::string("/decision/scan_center_angle"));
        node_->declare_parameter("back_target_state_pub_topic", std::string("/decision/back_target_state"));
        node_->declare_parameter("gesture_pub_topic", std::string("/decision/gesture"));
        node_->declare_parameter("enable_180_scan_topic", std::string("/decision/enable_180_scan"));
        node_->declare_parameter("is_target_at_home_topic", std::string("/decision/is_target_at_home"));

        // 1. 读取话题参数
        cmd_vel_topic_ = node_->get_parameter_or("cmd_vel_topic", std::string("/cmd_vel"));
        global_position_topic_ = node_->get_parameter_or("global_position_topic", std::string("/global_costmap/published_footprint"));
        cmd_vel_remap_topic_ = node_->get_parameter_or("cmd_vel_remap_topic", std::string("/cmd_vel_remap"));
        recv_topic_ = node_->get_parameter_or("recv_topic", std::string("/vision_recv_data"));
        send_topic_ = node_->get_parameter_or("send_topic", std::string("/vision_send_data"));
        back_target_topic_ = node_->get_parameter_or("back_target_topic", std::string("/back_target"));
        spin_control_topic_ = node_->get_parameter_or("spin_control_topic", std::string("/decision/spin_control"));
        scan_center_angle_topic_ = node_->get_parameter_or("scan_center_angle_topic", std::string("/decision/scan_center_angle"));
        back_target_state_pub_topic_ = node_->get_parameter_or("back_target_state_pub_topic", std::string("/decision/back_target_state"));
        gesture_pub_topic_ = node_->get_parameter_or("gesture_pub_topic", std::string("/decision/gesture"));
        enable_180_scan_topic_ = node_->get_parameter_or("enable_180_scan_topic", std::string("/decision/enable_180_scan"));
        pursue_stay_circle_radius_ = node_->get_parameter_or("pursue_stay_circle_radius", 0.5); // 追击静止小陀螺的半径，默认0.5米
        is_target_at_home_topic_ = node_->get_parameter_or("is_target_at_home_topic", std::string("/decision/is_target_at_home"));

        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] cmd_vel_topic: %s%s",
                    GREEN.c_str(), cmd_vel_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] global_position_topic: %s%s",
                    GREEN.c_str(), global_position_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] cmd_vel_remap_topic: %s%s",
                    GREEN.c_str(), cmd_vel_remap_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] spin_control_topic: %s%s",
                    GREEN.c_str(), spin_control_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] recv_topic: %s%s",
                    GREEN.c_str(), recv_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] send_topic: %s%s",
                    GREEN.c_str(), send_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] back_target_topic: %s%s",
                    GREEN.c_str(), back_target_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] scan_center_angle_topic: %s%s",
                    GREEN.c_str(), scan_center_angle_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] back_target_state_pub_topic: %s%s",
                    GREEN.c_str(), back_target_state_pub_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] gesture_pub_topic: %s%s",
                    GREEN.c_str(), gesture_pub_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] enable_180_scan_topic: %s%s",
                    GREEN.c_str(), enable_180_scan_topic_.c_str(), RESET.c_str());
        RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] is_target_at_home_topic: %s%s",
                    GREEN.c_str(), is_target_at_home_topic_.c_str(), RESET.c_str());

        // 2. 读取特殊区域相关参数 (在params的special_area空间下)
        std::vector<std::string> special_area_names = {
            "my_home_polygon", "my_move_polygon", "center_polygon", "opponent_home_polygon", "opponent_move_polygon"};
        for (const auto &name : special_area_names)
        {
            std::string param_name = "special_area." + name;
            node_->declare_parameter(param_name, std::vector<std::string>());
            auto goals_str = node_->get_parameter_or(param_name, std::vector<std::string>());

            if (!goals_str.empty())
            {
                std::vector<GlobalPose> goals;
                for (const auto &point_str : goals_str)
                {
                    GlobalPose gp;
                    size_t comma_pos = point_str.find(',');
                    if (comma_pos != std::string::npos)
                    {
                        gp.pose_x = std::stof(point_str.substr(0, comma_pos));
                        gp.pose_y = std::stof(point_str.substr(comma_pos + 1));
                        goals.push_back(gp);
                    }
                }
                special_area_from_params_[name] = goals;
                RCLCPP_INFO(node_->get_logger(), "%s[structure_get_params] Loaded %s: %zu points%s",
                            GREEN.c_str(), name.c_str(), goals.size(), RESET.c_str());
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "%s[structure_get_params] %s is empty or not set%s",
                             GREEN.c_str(), name.c_str(), RESET.c_str());
            }
        }
    }

    BT::NodeStatus PubRobotStatus::tick()
    {
        if (!is_init_)
        {
            return BT::NodeStatus::FAILURE;
        }

        std::unique_lock<std::mutex> lock(fsm_state_mutex_);

        // 1. 检查导航目标是否到达，如果到达则更新目标点队列
        // CheckIsReached_Update(0.2); // 内部不再加锁

        // 2. 根据状态机逻辑设置黑板变量
        //******************待尝试，直接用，不去用有限状态机去更新 *****************/
        // setOutput("need_supply", need_supply_);
        // setOutput("pose_from_human", is_pose_from_human_);
        // setOutput("is_pursue", is_pursue_);
        // setOutput("is_cruise", is_cruise_);
        // setOutput("is_in_upanddown_area", is_in_upanddown_area_);

        //RCLCPP_INFO(node_->get_logger(), "\033[36m[PubRobotStatus::tick] 进入tick函数\033[0m")
        setOutput("current_controller", "Omni");
        if (is_pose_from_human_)
        {
            FillGoalsDeque_Cruise();
            setOutput("is_in_upanddown_area", false);
            setOutput("pose_from_human", true);
            setOutput("need_supply", false);
            setOutput("is_pursue", false);
            setOutput("is_cruise", false);
            setOutput("human_goal", GlobalPose2PoseStamped(false));

            return BT::NodeStatus::SUCCESS;
        }
        if (is_need_supply_)
        {
            setOutput("need_supply", true);
            setOutput("pose_from_human", false);
            setOutput("is_pursue", false);
            setOutput("is_cruise", false);
            setOutput("supply_goal", GlobalPose2PoseStamped(false));

            hnurm_interfaces::msg::Gesture gesture_msg;
            gesture_msg.data = hnurm_interfaces::msg::Gesture::DEFEND;
            gesture_pub_->publish(gesture_msg);

            return BT::NodeStatus::SUCCESS;
        }
        if (is_pursue_)
        {
            setOutput("is_pursue", true);
            setOutput("pose_from_human", false);
            setOutput("need_supply", false);
            setOutput("is_cruise", false);
            setOutput("pursue_goal", GlobalPose2PoseStamped(true));

            hnurm_interfaces::msg::Gesture gesture_msg;
            gesture_msg.data = hnurm_interfaces::msg::Gesture::ATTACK;
            gesture_pub_->publish(gesture_msg);

            return BT::NodeStatus::SUCCESS;
        }
        if (is_cruise_)
        {
            setOutput("is_cruise", true);
            setOutput("pose_from_human", false);
            setOutput("need_supply", false);
            setOutput("is_pursue", false);
            setOutput("cruise_goal", GlobalPose2PoseStamped(false));

            hnurm_interfaces::msg::Gesture gesture_msg;
            if (current_my_area_ == SpecialAreaType::MY_HOME || current_my_area_ == SpecialAreaType::MY_MOVE)
            {
                gesture_msg.data = hnurm_interfaces::msg::Gesture::MOVE;
            }
            else
            {
                gesture_msg.data = hnurm_interfaces::msg::Gesture::ATTACK;
            }
            gesture_pub_->publish(gesture_msg);

            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    /***********************************************工具函数 start *********************************************************/

    /*  @name : CheckIsReached_Update
     *  @brief : 检查是否到达导航点，并更新目标点队列
     *  @param : current_pose 当前机器人位姿(来自订阅的导航话题)，current_goal_pose 当前导航点（来自队列top（））, threshold 到达阈值
     *  @return : bool 是否到达导航点
     */
    void PubRobotStatus::CheckIsReached_Update(float threshold)
    {
        // 被 tick() 或 fill_deque_timer_callback() 调用，调用者已持有 fsm_state_mutex_ 锁
        // 追击的优先级比云台打断输入和补给低，状态可能叠加，所以放在第二个检查
        if (is_pursue_ && !is_pose_from_human_ && !is_need_supply_) // 只有这样才表示是追击状态，不判断巡航是因为巡航优先级比追击低不需要比追击先判断
        {
            if (pursue_goal_deque_.empty())
            {
                RCLCPP_ERROR(node_->get_logger(), "[CheckIsReached_Update] 追击目标点队列为空，无法检查是否到达目标点");
                return;
            }
            GlobalPose current_goal_pose = pursue_goal_deque_.front(); // 获取当前目标点
            GlobalPose current_pose = current_x_y_;                    // 获取当前机器人坐标点，来自global_pose_callback
            float distance = std::sqrt(std::pow(current_pose.pose_x - current_goal_pose.pose_x, 2) +
                                       std::pow(current_pose.pose_y - current_goal_pose.pose_y, 2));
            // 距离阈值只判断一次，根据追击模式处理不同逻辑
            if (distance <= threshold)
            {
                switch (current_pursue_mode_)
                {
                case PursueMode::PURSUE:
                {
                    if (pursue_goal_deque_.size() > 1)
                    {
                        pursue_goal_deque_.pop_front();
                        RCLCPP_INFO(node_->get_logger(), "\033[34m[CheckIsReached_Update] 已到达【追击-PURSUE】目标点\033[0m");
                    }
                    else
                    {
                        RCLCPP_WARN(node_->get_logger(), "[CheckIsReached_Update] 已到达【追击-PURSUE】目标点，但追击队列已空或仅剩一个点，保持当前目标点不变");
                    }
                    return;
                }

                case PursueMode::STAY:
                {
                    // 追击目标点就是机器人当前位置，保持不动，无需处理
                    return;
                }

                case PursueMode::STAY_CIRCLE:
                {
                    pursue_goal_deque_.pop_front();
                    if (pursue_goal_deque_.size() < 3)
                    {
                        create_pursue_stay_circle_path();
                        RCLCPP_INFO(node_->get_logger(), "\033[34m[CheckIsReached_Update] 已到达【追击-STAY_CIRCLE】目标点，点数不足，重新装填\033[0m");
                    }
                    else
                    {
                        RCLCPP_INFO(node_->get_logger(), "\033[34m[CheckIsReached_Update] 已到达【追击-STAY_CIRCLE】目标点\033[0m");
                    }
                    return;
                }

                case PursueMode::FALLBACK:
                {
                    // TODO: 后退逻辑待实现
                    return;
                }

                default:
                {
                    RCLCPP_ERROR(node_->get_logger(), "[CheckIsReached_Update] 未知的追击模式");
                    return;
                }
                }
            }
            return;
        }

        if (current_cruise_goal_deque_.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "[CheckIsReached_Update] 巡航目标点队列为空，无法检查是否到达目标点");
            FillGoalsDeque_Cruise(); // 尝试重新装填巡航目标点队列
            return;
        }
        GlobalPose current_goal_pose = current_cruise_goal_deque_.front(); // 获取当前目标点
        GlobalPose current_pose = current_x_y_;                            // 获取当前机器人坐标点，来自global_pose_callback
        float distance = std::sqrt(std::pow(current_pose.pose_x - current_goal_pose.pose_x, 2) +
                                   std::pow(current_pose.pose_y - current_goal_pose.pose_y, 2));

        // 注意：虽然巡航，云台打断，补给公用一个逻辑，但是if的顺序就是决策树优先级：云台打断 > 补给 > 巡航，保证状态优先级正确
        if (is_pose_from_human_)
        {
            if (distance <= threshold) // 到达目标点
            {
                // 这里不弹出队列，因为这个目标点是来自云台手的实时输入，不是预设的巡航点，保持最新输入即可
                RCLCPP_INFO(node_->get_logger(), "\033[34m[CheckIsReached_Update] 已到达云台手目标点\033[0m");
            }
        }
        else if (is_need_supply_)
        {
            if (distance <= threshold) // 到达补给点
            {
                RCLCPP_INFO(node_->get_logger(), "\033[34m[CheckIsReached_Update] 已到达补给点\033[0m");
                // is_need_supply_ = false;  ！！！不能重置补给标志位，补给完成需要血量判断
            }
        }
        else if (is_cruise_)
        {
            if (distance <= threshold && current_cruise_mode_ != CruiseMode::SUPPLY) // 到达目标点且不是补给点或堡垒点
            {
                RCLCPP_INFO(node_->get_logger(), "\033[34m[CheckIsReached_Update] 到达巡航目标点，弹出 front，当前队列大小: %zu\033[0m", 
                            current_cruise_goal_deque_.size());
                current_cruise_goal_deque_.pop_front(); // 从队列中移除已到达的目标点
                RCLCPP_INFO(node_->get_logger(), "\033[34m[CheckIsReached_Update] 弹出后队列大小: %zu\033[0m", current_cruise_goal_deque_.size());
            }

            // 检查并持续队列
            if (current_cruise_goal_deque_.size() <= 2) // 每次tick检查是否需要重新装填队列
            {
                RCLCPP_INFO(node_->get_logger(), "\033[34m[CheckAndReFillGoalsDeque_Cruise] 巡航队列即将耗尽(当前大小: %zu)，自动重新装填\033[0m",
                            current_cruise_goal_deque_.size());
                FillGoalsDeque_Cruise();
            }
        }
        else
        {
        }
    }

    void PubRobotStatus::FillGoalsDeque_Cruise()
    {
        // mode : [SUPPLY, CENTER, AMBUSH, INIT]
        RCLCPP_INFO(node_->get_logger(), "\033[33m[FillGoalsDeque_Cruise] 开始填充队列，当前模式: %s, 原队列大小: %zu\033[0m", 
                    cruiseModeToString(current_cruise_mode_).c_str(), current_cruise_goal_deque_.size());
        
        current_cruise_goal_deque_.clear();
        
        if (current_cruise_mode_ == CruiseMode::SUPPLY)
        {
            // 补给模式只装填补给点
            auto it = path_pose_arrays_.find(CruiseMode::SUPPLY);
            if (it == path_pose_arrays_.end() || it->second.empty()) {
                RCLCPP_ERROR(node_->get_logger(), "\033[31m[FillGoalsDeque_Cruise] SUPPLY 模式路径点未找到或为空！\033[0m");
            } else {
                for (const auto &goal : it->second)
                {
                    current_cruise_goal_deque_.push_back(goal);
                }
                RCLCPP_INFO(node_->get_logger(), "\033[32m[FillGoalsDeque_Cruise] SUPPLY 模式已填充 %zu 个目标点\033[0m", 
                            current_cruise_goal_deque_.size());
            }
        }
        else
        {
            auto it = path_pose_arrays_.find(current_cruise_mode_);
            if (it == path_pose_arrays_.end() || it->second.empty()) {
                RCLCPP_ERROR(node_->get_logger(), "\033[31m[FillGoalsDeque_Cruise] 模式 %s 的路径点未找到或为空！path_pose_arrays_ 大小: %zu\033[0m",
                             cruiseModeToString(current_cruise_mode_).c_str(), path_pose_arrays_.size());
                // 打印所有可用的模式
                std::string available_modes;
                for (const auto& pair : path_pose_arrays_) {
                    available_modes += cruiseModeToString(pair.first) + "(" + std::to_string(pair.second.size()) + ") ";
                }
                RCLCPP_ERROR(node_->get_logger(), "\033[31m[FillGoalsDeque_Cruise] 可用模式: %s\033[0m", available_modes.c_str());
            } else {
                for (int i = 0; i < 3; ++i)
                {
                    for (const auto &goal : it->second)
                    {
                        current_cruise_goal_deque_.push_back(goal);
                    }
                }
                RCLCPP_INFO(node_->get_logger(), "\033[32m[FillGoalsDeque_Cruise] 模式 %s 已填充 %zu 个目标点 (原路径点数: %zu, 重复3次)\033[0m", 
                            cruiseModeToString(current_cruise_mode_).c_str(), current_cruise_goal_deque_.size(), it->second.size());
            }
        }
    }

    void PubRobotStatus::FillGoalsDeque_Pursue()
    {
        pursue_goal_deque_.clear();

        // 1. 找到目标点在地图坐标系下的位置，判断是不是在泉水
        GlobalPose target_pose;
        geometry_msgs::msg::Pose target_in_map = get_targetpose_in_map(send_target_information_.target_distance);

        target_pose.pose_x = target_in_map.position.x;
        target_pose.pose_y = target_in_map.position.y;
            
        current_target_area_ = GetPointArea(target_pose);

        // 2. 发布目标点是否在泉水的消息
        std_msgs::msg::Bool is_target_at_home_msg;
        if(current_target_area_ == SpecialAreaType::OPPONENT_HOME)
            is_target_at_home_msg.data = true;
        else
            is_target_at_home_msg.data = false;
        is_target_at_home_pub_->publish(is_target_at_home_msg);

        // 3. pushback目标点
        if (current_pursue_mode_ == PursueMode::PURSUE)
        {
            // 在泉水，追击目标点就是机器人当前位置，保持不动
            // 不在泉水，追击目标点就是目标点位置
            if (current_target_area_ == SpecialAreaType::OPPONENT_HOME){
                pursue_goal_deque_.push_back(current_x_y_);
            }else{
                pursue_goal_deque_.push_back(target_pose);
            }
                
        }
        else if (current_pursue_mode_ == PursueMode::STAY)
        {
            // STAY追击目标点就是机器人当前位置，保持不动
            pursue_goal_deque_.push_back(current_x_y_);
            if (pursue_goal_deque_.size() > 50)
            {
                return; // 已经装填过了，不需要重复装填了，避免频繁调用GetPointArea函数
            }
        }
        else if (current_pursue_mode_ == PursueMode::STAY_CIRCLE)
        {
            create_pursue_stay_circle_path();
        }
        else
        {
        }
    }

    void PubRobotStatus::UpdateCurrentCruiseMode()
    {
        previous_cruise_mode_ = current_cruise_mode_;
        switch (current_cruise_mode_)
        {
        case CruiseMode::INIT:
            if(recv_robot_information_.remain_time > 150.0){ // 前半程时间
                if (center_occupy_status_ == CenterOccupyStatus::NONE)
                    current_cruise_mode_ = CruiseMode::CENTER;
                else
                    current_cruise_mode_ = CruiseMode::AMBUSH;
            }
            else{
                current_cruise_mode_ = CruiseMode::CENTER;
            }
            
            break;
        case CruiseMode::CENTER:
            // 我方占领：AMBUSH 双方占领：CENTER 敌方占领：GUARD
            if (recv_robot_information_.remain_time > 150.0)
            { // 前半程时间
                if (center_occupy_status_ == CenterOccupyStatus::NONE || center_occupy_status_ == CenterOccupyStatus::OPPONENT)
                    current_cruise_mode_ = CruiseMode::CENTER;
                else
                    current_cruise_mode_ = CruiseMode::AMBUSH;
            }
            else
            {
                if(center_occupy_status_ == CenterOccupyStatus::MYSELF){
                    current_cruise_mode_ = CruiseMode::AMBUSH;
                }
                else
                {
                    current_cruise_mode_ = CruiseMode::CENTER;
                }
            }
            break;
        case CruiseMode::AMBUSH:
            // 我方占领：AMBUSH 双方占领：CENTER 敌方占领：CENTER
            if (center_occupy_status_ == CenterOccupyStatus::OPPONENT || center_occupy_status_ == CenterOccupyStatus::BOTH)
                current_cruise_mode_ = CruiseMode::CENTER;
            else
                current_cruise_mode_ = CruiseMode::AMBUSH;
            break;
        case CruiseMode::SUPPLY:
            break; // 补给模式不自动切换，保持当前模式，直到状态机逻辑切换到其他模式
        case CruiseMode::GUARD:
            // 我方占领：AMBUSH 双方占领：CENTER 敌方占领：GUARD
            if (center_occupy_status_ == CenterOccupyStatus::MYSELF)
                current_cruise_mode_ = CruiseMode::AMBUSH;
            else if (center_occupy_status_ == CenterOccupyStatus::OPPONENT)
                current_cruise_mode_ = CruiseMode::GUARD;
            else
                current_cruise_mode_ = CruiseMode::CENTER;
            break;
        case CruiseMode::HOME:
            current_cruise_mode_ = CruiseMode::GUARD;
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "[UpdateCurrentCruiseMode] 未知的巡航模式");
            current_cruise_mode_ = CruiseMode::INIT;
            break;
        }
    }

    PubRobotStatus::SpecialAreaType PubRobotStatus::GetPointArea(const GlobalPose &point)
    {
        // 注意这里的顺序，为了避免有洞区域的复杂判断，对洞内区域(双方堡垒)先判断，是的话直接返回，不是的话才会判断是不是属于其他区域（包括套着他的区域（双方半场））
        // 顺序：1. 双方堡垒 2. 双方起伏路段 3. 高地 4. 双方半场
        std::vector<SpecialAreaType> check_order = {
            SpecialAreaType::CENTER,
            SpecialAreaType::MY_MOVE,
            SpecialAreaType::OPPONENT_MOVE,
            SpecialAreaType::OPPONENT_HOME,
            SpecialAreaType::MY_HOME,
        };
        for (auto type : check_order)
        {
            if (is_in_this_polygon(point, special_area_poses_[type]))
            {
                return type;
            }
        }
        return SpecialAreaType::ILLEGAL;
    }

    /* @name : GlobalPose2PoseStamped
     *  @brief : 将全局坐标点转换为PoseStamped消息
     *  @param : global_pose 全局坐标点
     *  @return : pose_stamped PoseStamped消息
     */
    geometry_msgs::msg::PoseStamped PubRobotStatus::GlobalPose2PoseStamped(bool is_pursue)
    {
        //RCLCPP_INFO(node_->get_logger(), "\033[36m进入GlobalPose2PoseStamped\033[0m");
        GlobalPose global_pose;
        if (is_pursue)
        {
            if (pursue_goal_deque_.empty())
            {
                RCLCPP_ERROR(node_->get_logger(), "[tick] Pursue goal deque is empty!");
                {
                    // tick() 或 fill_deque_timer_callback() 已持有 fsm_state_mutex_ 锁
                    global_pose = current_x_y_; // 使用当前位置作为备选
                }
            }
            else
            {
                global_pose = pursue_goal_deque_.front(); // 获取当前追击目标点
            }
            if(is_pursue && current_pursue_mode_ == PursueMode::STAY){
                global_pose = current_x_y_; // STAY模式追击目标点就是机器人当前位置，保持不动
            }
        }
        else
        {
            if (current_cruise_goal_deque_.empty())
            {
                RCLCPP_ERROR(node_->get_logger(), "\033[31m[GlobalPose2PoseStamped] Cruise goal deque is empty! 使用当前位置作为备选。当前巡航模式: %s\033[0m",
                             cruiseModeToString(current_cruise_mode_).c_str());
                {
                    // tick() 或 fill_deque_timer_callback() 已持有 fsm_state_mutex_ 锁
                    global_pose = current_x_y_; // 使用当前位置作为备选
                }
            }
            else
            {
                global_pose = current_cruise_goal_deque_.front(); // 获取当前巡航目标点
                // RCLCPP_INFO(node_->get_logger(), "\033[36m[GlobalPose2PoseStamped] 获取巡航目标点: (%f, %f), 当前模式: %s, 队列大小: %zu\033[0m",
                //             global_pose.pose_x, global_pose.pose_y, 
                //             cruiseModeToString(current_cruise_mode_).c_str(), current_cruise_goal_deque_.size());
            }
        }
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = node_->now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = global_pose.pose_x;
        pose_stamped.pose.position.y = global_pose.pose_y;
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;
        return pose_stamped;
    }

    void PubRobotStatus::ScanCenterAngleProcess(const double target_yaw_deg)
    {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        double current_yaw_deg = recv_robot_information_.yaw;

        // 计算差值，这里是多圈角度差值，使用std::remainder函数将结果限制在[-180, 180]范围内，确保旋转方向正确
        double yaw_diff_deg = std::remainder(target_yaw_deg - current_yaw_deg, 360.0);
        // 打印验证
        //RCLCPP_INFO_THROTTLE(node_->get_logger(), steady_clock, 3000, "\033[36m[fill_deque_timer_callback]控制180度扫描，当前需要旋转的角度是：yaw_diff_deg: %f\033[0m", yaw_diff_deg);
        std_msgs::msg::Float32 scan_center_angle;
        // 多圈加上需要转的角度，发布出去
        scan_center_angle.data = current_yaw_deg + yaw_diff_deg;
        scan_center_angle_pub_->publish(scan_center_angle);
        //RCLCPP_INFO_THROTTLE(node_->get_logger(), steady_clock, 3000, "\033[36m[fill_deque_timer_callback]控制180度扫描，目标多圈角度是：scan_center_angle.data: %f\033[0m", scan_center_angle.data);
    }

    PubRobotStatus::CenterOccupyStatus PubRobotStatus::GetCenterOccupyStatus(float msg_num)
    {
        if (msg_num == 0.0)
            return CenterOccupyStatus::NONE;
        else if (msg_num == 1.0)
            return CenterOccupyStatus::MYSELF;
        else if (msg_num == 2.0)
            return CenterOccupyStatus::OPPONENT;
        else if (msg_num == 3.0)
            return CenterOccupyStatus::BOTH;
        else
            return CenterOccupyStatus::BOTH;
    }

    /* @name: calculateAverage
     *  @brief：用于计算多边形点集的平均坐标，作为机器人当前位置的估计值。将定位系统发布的 footprint（多边形）转换为机器人的中心点坐标
     *  @param：msg geometry_msgs::msg::PolygonStamped msg - 包含多边形点集的 ROS 消息
     */
    PubRobotStatus::GlobalPose PubRobotStatus::calculateAverage(geometry_msgs::msg::PolygonStamped &msg)
    {
        double x_sum = 0.0;
        double y_sum = 0.0;
        GlobalPose cgp_;
        for (auto const &p : msg.polygon.points)
        {
            x_sum += p.x;
            y_sum += p.y;
        }
        cgp_.pose_x = x_sum / msg.polygon.points.size();
        cgp_.pose_y = y_sum / msg.polygon.points.size();
        return cgp_;
    }

    geometry_msgs::msg::Pose PubRobotStatus::get_targetpose_in_map(float &distance)
    {
        geometry_msgs::msg::TransformStamped map_to_camera;
        /**/ auto start_time = std::chrono::steady_clock::now();
        try
        {
            // 使用 50ms 前的时间戳，允许 TF 有一定延迟；超时设为 10ms，快速返回
            rclcpp::Time target_time = node_->now() - rclcpp::Duration::from_seconds(0.05);
            map_to_camera = tf_buffer_->lookupTransform(
                "map",
                "camera_link",
                target_time,
                rclcpp::Duration::from_seconds(0.01) // 10ms 超时，快速失败
            );
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to lookup transform【map -> camera_link】: %s", ex.what());
            geometry_msgs::msg::Pose invalid_pose;
            invalid_pose.position.x = NAN; // 标记为无效
            return invalid_pose;
        }

        geometry_msgs::msg::Pose target_in_camera;
        target_in_camera.position.x = distance;
        target_in_camera.position.y = 0.0; // y轴正前方
        target_in_camera.position.z = 0.0;
        target_in_camera.orientation.w = 1.0; // 旋转不变（与camera同向）
        target_in_camera.orientation.x = 0.0;
        target_in_camera.orientation.y = 0.0;
        target_in_camera.orientation.z = 0.0;

        // 变换到 map 系
        geometry_msgs::msg::Pose target_in_map;
        tf2::doTransform(target_in_camera, target_in_map, map_to_camera); // target_map = T_map_cam × target_cam

        /**/ auto end_time = std::chrono::steady_clock::now();
        /**/ auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        /**/ RCLCPP_INFO(node_->get_logger(), "\033[36m找到map->camera变换并发布目标TF用时: %ld ms\033[0m", duration);
        return target_in_map;
    }

    void PubRobotStatus::create_pursue_stay_circle_path()
    {
        pursue_goal_deque_.clear(); // 清空当前追击目标队列

        if (current_pursue_mode_ != PursueMode::STAY_CIRCLE || send_target_information_.target_distance < 0.5)
        {
            pursue_goal_deque_.push_back(current_x_y_); // 象征性的添加，即使pop掉了，加入goal的函数GlobalPose2PoseStamped为空会发布自己现在的座标
            return;                                     // 只有在切换到追击静止小陀螺模式且目标距离足够远时才生成路径
        }

        // 以目标点为圆心，半径为 pursue_stay_circle_radius_，生成若干个均匀分布的点作为追击静止小陀螺的路径
        int num_points = 8;                             // 圆周上点的数量
        double angle_increment = 2 * M_PI / num_points; // 每个点之间的角度增量

        for (int times = 0; times < 3; ++times) // 重复生成多圈，增加持续时间
        {
            for (int i = 0; i < num_points; ++i)
            {
                double angle = i * angle_increment;
                GlobalPose circle_point;
                circle_point.pose_x = current_x_y_.pose_x + pursue_stay_circle_radius_ * cos(angle);
                circle_point.pose_y = current_x_y_.pose_y + pursue_stay_circle_radius_ * sin(angle);
                pursue_goal_deque_.push_back(circle_point);
            }
        }
    }

    bool PubRobotStatus::is_in_this_polygon(const GlobalPose &point, const std::vector<GlobalPose> &polygon)
    {
        const float EPSILON = 1e-9;
        bool inside = false;
        int n = polygon.size();

        for (int i = 0, j = n - 1; i < n; j = i++)
        {
            const auto &pi = polygon[i];
            const auto &pj = polygon[j];

            // 射线法核心判断
            if (((pi.pose_y > point.pose_y) != (pj.pose_y > point.pose_y)) &&
                (point.pose_x < (pj.pose_x - pi.pose_x) * (point.pose_y - pi.pose_y) / (pj.pose_y - pi.pose_y + EPSILON) + pi.pose_x))
            {
                inside = !inside;
            }
        }
        return inside;
    }

    float PubRobotStatus::calculate_distance(const GlobalPose &pose1, const GlobalPose &pose2)
    {
        return std::sqrt(std::pow(pose1.pose_x - pose2.pose_x, 2) + std::pow(pose1.pose_y - pose2.pose_y, 2));
    }

    PubRobotStatus::CruiseMode PubRobotStatus::stringToCruiseMode(const std::string &mode_str)
    {
        if (mode_str == "initial_goals")
            return CruiseMode::INIT;
        if (mode_str == "center_goals")
            return CruiseMode::CENTER;
        if (mode_str == "supply_goals")
            return CruiseMode::SUPPLY;
        if (mode_str == "ambush_goals")
            return CruiseMode::AMBUSH;
        if (mode_str == "guard_goals")
            return CruiseMode::GUARD;
        if (mode_str == "home_goals")
            return CruiseMode::HOME;
        return CruiseMode::INIT; // 默认返回
    }

    std::string PubRobotStatus::cruiseModeToString(CruiseMode mode)
    {
        switch (mode)
        {
        case CruiseMode::INIT:
            return "INIT";
        case CruiseMode::CENTER:
            return "CENTER";
        case CruiseMode::SUPPLY:
            return "SUPPLY";
        case CruiseMode::AMBUSH:
            return "AMBUSH";
        case CruiseMode::GUARD:
            return "GUARD";
        case CruiseMode::HOME:
            return "HOME";
        }
        return "unknown";
    }

    std::string PubRobotStatus::specialAreaToString(SpecialAreaType area)
    {
        switch (area)
        {
        case SpecialAreaType::MY_HOME:
            return "我方基地";
        case SpecialAreaType::MY_MOVE:
            return "我方移动区";
        case SpecialAreaType::CENTER:
            return "中心";
        case SpecialAreaType::OPPONENT_HOME:
            return "敌方基地";
        case SpecialAreaType::OPPONENT_MOVE:
            return "敌方移动区";
        case SpecialAreaType::ILLEGAL:
            return "非法区域";
        default:
            return "未知";
        }
    }
    /***********************************************工具函数 end *********************************************************/

    /**************************回调函数 start*********************************/

    /* @name : global_pose_callback 10ms（不确定需要测试）
     * @brief : 全局定位回调函数,填充现在机器人坐标
     * @param : msg 全局定位消息
     * @return : None
     * @周期 : ~20ms (50Hz，由全局定位系统发布频率决定)
     * @执行时间 : ~1-5μs (计算平均值+区域检测)
     */
    void PubRobotStatus::global_pose_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
    {
        if (!is_init_)
            return;
        std::lock_guard<std::mutex> lock(fsm_state_mutex_);
        current_x_y_ = calculateAverage(*msg);
        current_my_area_ = GetPointArea(current_x_y_);
    }

    /* 周期：10ms 100Hz
     * @name : remap_cmd_vel_callback
     * @brief : 速度命令回调重映射函数,起伏路段自控制，后时旋转180,其他情况不变
     * @param : msg 速度命令消息
     * @周期 : 10ms (100Hz)
     * @执行时间 : ~1-5μs (普通情况) / ~0.5-10ms (起伏路段，含TF查询)
     */
    void PubRobotStatus::remap_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // ros2 topic hz /cmd_vel
        //  减速
        if (!is_init_)
            return;
        geometry_msgs::msg::Twist vel_pub_;
        vel_pub_ = *msg;

        cmd_vel_remap_pub_->publish(vel_pub_);
    }

    /* @周期 : 37ms (串口1000Hz，但我们只处理50Hz的消息，过快的消息会被节流掉)
     * @执行时间 : ~1μs (简单状态判断)
     */
    void PubRobotStatus::recv_callback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg)
    {
        auto now = std::chrono::steady_clock::now();
        if (now - last_recv_time_ < std::chrono::milliseconds(37))
        {
            return;
        }
        last_recv_time_ = now;


        // 处理串口状态消息，更新内部状态变量
        recv_robot_information_ = *msg;

        last_center_occupy_status_ = center_occupy_status_;
        center_occupy_status_ = GetCenterOccupyStatus(recv_robot_information_.center_ctrl);

        if (!is_color_setted_ && recv_robot_information_.self_color.data != 0)
        {
            self_color_ = recv_robot_information_.self_color.data == 1 ? SelfColor::RED : SelfColor::BLUE;
            is_color_setted_ = true; // 颜色信息已设置
        }

        std::unique_lock<std::mutex> lock(fsm_state_mutex_); // 加锁保护

        if (recv_robot_information_.current_hp <= 180 || recv_robot_information_.allow_fire_amount <= 50)
        {
            is_need_supply_ = true; // 需要补给

            previous_cruise_mode_ = current_cruise_mode_; // 记录切换前的巡航模式
            current_cruise_mode_ = CruiseMode::SUPPLY;    // 切换到补给模式
        }
        else if (recv_robot_information_.current_hp > 360 && recv_robot_information_.allow_fire_amount > 100)
        {
            if(is_need_supply_){
                is_need_supply_ = false; // 关闭补给标志

                previous_cruise_mode_ = current_cruise_mode_; // 记录切换前的巡航模式
                // todo: 加血之后，根据串口状态判断是去中心还是去阻击，还是回到我方拐角处
                if (center_occupy_status_ == CenterOccupyStatus::MYSELF)
                {
                    current_cruise_mode_ = CruiseMode::AMBUSH; // 切换回我方拐角
                }
                else if (center_occupy_status_ == CenterOccupyStatus::OPPONENT || center_occupy_status_ == CenterOccupyStatus::BOTH)
                {
                    current_cruise_mode_ = CruiseMode::GUARD; // 切换回中心
                }
                else{
                    current_cruise_mode_ = CruiseMode::CENTER; // 默认切换回中心
                }
                // ！！！！！！！！！！！！注意更新start_time_ ！！！！！！！！！！！！
                current_cruise_mode_start_time_ = node_->now();
            }       
        }
        else
        {
        }
    }
    /* @周期 : 未知
     * @执行时间 : ~0.5μs (简单条件判断)
     */
    void PubRobotStatus::back_target_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        std::unique_lock<std::mutex> lock(fsm_state_mutex_);
        if (is_pursue_ || is_need_supply_ || is_pose_from_human_)
            return; // 这些状态优先级最高，正在追击时后视相机信息不处理，追击也不处理

        if (msg->data != 666.0 && std::abs(msg->data) < 65.0)
        { // 666.0是后视相机无效数据的标志，其他小于3.0的数值表示检测到后方目标且距离较近{
            is_back_target_detected_ = true;
        }
        else
        {
            is_back_target_detected_ = false;
        }
    }

    /* @周期 : 20ms
     * @执行时间 : ~1-5μs (状态切换逻辑)
     */
    void PubRobotStatus::send_target_info_callback(const hnurm_interfaces::msg::VisionSendData::SharedPtr msg)
    {
        if (!is_init_)
            return;
        // 处理视觉系统发送的目标状态信息，控制机器人行为模式切换
        send_target_information_ = *msg;

        std::unique_lock<std::mutex> lock(fsm_state_mutex_); // 加锁保护

        if (send_target_information_.target_state.data > 0 && recv_robot_information_.remain_time < 240.0) // 除了0(无目标)、2(工程)、6(前哨)以外的其他目标类型，且视觉状态为1（表示检测到目标）
        {
            current_cruise_mode_start_time_ = node_->now();
            if (send_target_information_.target_distance < 8.0) // 近距离目标且比赛时间充足
            {
                if(current_my_area_ == SpecialAreaType::CENTER){
                    current_pursue_mode_ = PursueMode::STAY; // 近距离目标且在敌方基地，使用追击静止小陀螺模式
                }
                is_cruise_ = false;                      // 不巡航
                is_pursue_ = true;                       // 追击
            }
            else
            {
                current_pursue_mode_ = PursueMode::PURSUE; // 远距离目标，正常追击
                is_cruise_ = false;                        // 不巡航
                is_pursue_ = true;                         // 追击
            }
            FillGoalsDeque_Pursue(); // 根据追击模式更新目标点队列
        }
        else
        {
            is_cruise_ = true;  // 巡航
            is_pursue_ = false; // 不追击
        }
    }

    /* @周期 : 7s
     * @执行时间 : ~10-50μs (巡航模式更新逻辑)
     */
    void PubRobotStatus::cruise_change_timer_callback()
    {
        if (!is_init_)
            return;
        RCLCPP_INFO(node_->get_logger(), "\033[32m[cruise_change_timer_callback] 进入巡航模式切换定时器回调\033[0m");
        rclcpp::Time now = node_->now();
        if (now - current_cruise_mode_start_time_ > change_cruise_mode_time_threshold_)
        {
            RCLCPP_INFO(node_->get_logger(), "\033[32m[cruise_change_timer_callback] 巡航模式自动切换触发\033[0m");
            UpdateCurrentCruiseMode();
            current_cruise_mode_start_time_ = now;
        }
    }

    /* @周期 : 223ms (5Hz)
     * @执行时间 : ~50-100μs (区域检测+距离计算+日志输出)
     * @说明 : 可能是最耗时的回调，包含大量日志输出
     */
    void PubRobotStatus::special_area_timer_callback()
    {
        {
            std::lock_guard<std::mutex> lock(fsm_state_mutex_);

            if (is_color_setted_ && !is_init_)
            {
                initialization(); // 省一个定时器，就在这里循环判断初始化
            }
            if (!is_init_)
                return; // 未初始化完成前不执行后续逻辑

            std_msgs::msg::Float32 spin_control_msg_;

            switch (current_my_area_)
            {
            case SpecialAreaType::MY_HOME:
                spin_control_msg_.data = 4.0; // 快速移动不小陀螺
                spin_control_pub_->publish(spin_control_msg_);
                break;
            case SpecialAreaType::MY_MOVE:
                spin_control_msg_.data = 1.0; // 快速移动不小陀螺
                spin_control_pub_->publish(spin_control_msg_);
                break;
            case SpecialAreaType::OPPONENT_MOVE:
                spin_control_msg_.data = 1.0; // 小陀螺
                spin_control_pub_->publish(spin_control_msg_);
                break;
            case SpecialAreaType::CENTER:
                spin_control_msg_.data = 1.0; // 小陀螺
                spin_control_pub_->publish(spin_control_msg_);
                break;
            case SpecialAreaType::OPPONENT_HOME:
                spin_control_msg_.data = 1.0; // 小陀螺
                spin_control_pub_->publish(spin_control_msg_);
                break;
            case SpecialAreaType::ILLEGAL:
                spin_control_msg_ = last_spin_control_msg_; // 非法区域保持上次的旋转控制状态，避免频繁切换
                spin_control_pub_->publish(spin_control_msg_);
                break;
            }
            spin_control_pub_->publish(spin_control_msg_);
            last_spin_control_msg_ = spin_control_msg_;

            std_msgs::msg::Bool enable_180_scan_msg_;
            // switch (current_cruise_mode_)
            // {
            // case CruiseMode::INIT:
            //     enable_180_scan_msg_.data = true;
            //     enable_180_scan_pub_->publish(enable_180_scan_msg_);
            //     ScanCenterAngleProcess(45.0);
            //     break;
            // case CruiseMode::CENTER:
            //     enable_180_scan_msg_.data = true;
            //     enable_180_scan_pub_->publish(enable_180_scan_msg_);
            //     ScanCenterAngleProcess(45.0);
            //     break;
            // case CruiseMode::AMBUSH:
            //     enable_180_scan_msg_.data = true;
            //     enable_180_scan_pub_->publish(enable_180_scan_msg_);
            //     ScanCenterAngleProcess(-45.0);
            //     break;
            // case CruiseMode::GUARD:
            //     enable_180_scan_msg_.data = true;
            //     enable_180_scan_pub_->publish(enable_180_scan_msg_);
            //     ScanCenterAngleProcess(45.0);
            //     break;
            // case CruiseMode::HOME:
            //     enable_180_scan_msg_.data = true;
            //     enable_180_scan_pub_->publish(enable_180_scan_msg_);
            //     ScanCenterAngleProcess(-90.0);
            //     break;
            // case CruiseMode::SUPPLY:
            //     enable_180_scan_msg_.data = true;
            //     ScanCenterAngleProcess(-90.0);
            //     enable_180_scan_pub_->publish(enable_180_scan_msg_);
            //     break;
            // default:
            //     enable_180_scan_msg_.data = false;
            //     enable_180_scan_pub_->publish(enable_180_scan_msg_);
            //     break;
            // }
            enable_180_scan_msg_.data = false;
            enable_180_scan_pub_->publish(enable_180_scan_msg_);

            rclcpp::Clock steady_clock(RCL_STEADY_TIME);
            RCLCPP_INFO_THROTTLE(node_->get_logger(), steady_clock, 1500, "\033[35m[总信息输出]--------------------------------------\033[0m");
            RCLCPP_INFO_THROTTLE(node_->get_logger(), steady_clock, 1500, "\033[35m[总信息输出] 补给:%d 追击:%d 巡航:%d 后视:%d\033[0m", is_need_supply_, is_pursue_, is_cruise_, is_back_target_detected_);
            RCLCPP_INFO_THROTTLE(node_->get_logger(), steady_clock, 1500, "\033[35m[总信息输出]当前区域: %s\033[0m", specialAreaToString(current_my_area_).c_str());
            RCLCPP_INFO_THROTTLE(node_->get_logger(), steady_clock, 1500, "\033[35m[总信息输出]当前巡航模式: %s\033[0m", cruiseModeToString(current_cruise_mode_).c_str());
            RCLCPP_INFO_THROTTLE(node_->get_logger(), steady_clock, 1500,
                                 "\033[35m[总信息输出] 小陀螺:%.1f     180度扫描:%s \033[0m",
                                 last_spin_control_msg_.data,
                                 enable_180_scan_msg_.data ? "开" : "关");
            RCLCPP_INFO_THROTTLE(node_->get_logger(), steady_clock, 1500, "\033[35m[总信息输出]--------------------------------------\033[0m");
        }       
    }

    /* @周期 : 4ms (250Hz)
     * @执行时间 : ~10-20μs (目标点队列更新逻辑)
     */
    void PubRobotStatus::fill_deque_timer_callback()
    {

        std::lock_guard<std::mutex> lock(fsm_state_mutex_);
        CheckIsReached_Update(0.25);
        if ((current_cruise_mode_ == CruiseMode::AMBUSH || current_cruise_mode_ == CruiseMode::GUARD) && last_center_occupy_status_ != center_occupy_status_)
        {
            // 巡航：center->ambush/guard，离开中心区域了，如果中心区域现在是被对方占领了或者无人占领了，就切换回中心巡航模式
            // 也是AMBUSH/GUARD模式下，如果中心易主了，就切换回中心巡航模式
            if(last_center_occupy_status_ == CenterOccupyStatus::MYSELF && center_occupy_status_ != CenterOccupyStatus::MYSELF)
            {
                previous_cruise_mode_ = current_cruise_mode_;
                current_cruise_mode_ = CruiseMode::CENTER;
                current_cruise_mode_start_time_ = node_->now(); // 更新模式开始时间，避免模式切换过快
                RCLCPP_INFO(node_->get_logger(), "\033[34m[fill_deque_timer_callback] AMBUSH/GUARD模式下离开中心区域但中心区域只有我自己，要回到中心：巡航模式切换: %s -> %s\033[0m", cruiseModeToString(previous_cruise_mode_).c_str(), cruiseModeToString(current_cruise_mode_).c_str());
            }
            // 巡航：center->ambush/guard，离开中心区域了，如果中心区域现在是被对方占领了或者无人占领了，就切换回中心巡航模式
            // 也是AMBUSH/GUARD模式下，如果中心易主了，就切换回中心巡航模式
            if (last_center_occupy_status_ == CenterOccupyStatus::BOTH && (center_occupy_status_ == CenterOccupyStatus::NONE || center_occupy_status_ == CenterOccupyStatus::OPPONENT))
            {
                previous_cruise_mode_ = current_cruise_mode_;
                current_cruise_mode_ = CruiseMode::CENTER;
                current_cruise_mode_start_time_ = node_->now(); // 更新模式开始时间，避免模式切换过快
                RCLCPP_INFO(node_->get_logger(), "\033[34m[fill_deque_timer_callback] AMBUSH/GUARD模式下离开中心区域但中心区域只有我自己，要回到中心：巡航模式切换: %s -> %s\033[0m", cruiseModeToString(previous_cruise_mode_).c_str(), cruiseModeToString(current_cruise_mode_).c_str());
            }
            // 巡航：ambush->center，我方在敌方在中心的时候进入中心区域了，如果中心区域现在是被双方占领了，就切换回中心巡航模式
            if(last_center_occupy_status_ == CenterOccupyStatus::OPPONENT && center_occupy_status_ == CenterOccupyStatus::BOTH)
            {
                previous_cruise_mode_ = current_cruise_mode_;
                current_cruise_mode_ = CruiseMode::CENTER;
                current_cruise_mode_start_time_ = node_->now(); // 更新模式开始时间，避免模式切换过快
                RCLCPP_INFO(node_->get_logger(), "\033[34m[fill_deque_timer_callback] AMBUSH/GUARD模式下离开中心区域但中心区域被对方占领了，要回到中心：巡航模式切换: %s -> %s\033[0m", cruiseModeToString(previous_cruise_mode_).c_str(), cruiseModeToString(current_cruise_mode_).c_str());
            }
            if (last_center_occupy_status_ == CenterOccupyStatus::OPPONENT && center_occupy_status_ != CenterOccupyStatus::BOTH){
                // 敌方opponent->非both（my，none）
                if (current_cruise_mode_ == CruiseMode::AMBUSH)
                {
                    // 保持ambush攻击堵住家
                }
                else
                {
                    // GUARD模式下，我方重新占领，切换回中心巡航模式
                    previous_cruise_mode_ = current_cruise_mode_;
                    current_cruise_mode_ = CruiseMode::CENTER;
                    current_cruise_mode_start_time_ = node_->now(); // 更新模式开始时间，避免模式切换过快
                    RCLCPP_INFO(node_->get_logger(), "\033[34m[fill_deque_timer_callback] AMBUSH/GUARD模式下离开中心区域但中心区域被对方占领了，要回到中心：巡航模式切换: %s -> %s\033[0m", cruiseModeToString(previous_cruise_mode_).c_str(), cruiseModeToString(current_cruise_mode_).c_str());
                }
            }          
        }
        if (current_cruise_mode_ != previous_cruise_mode_)
        {
            RCLCPP_INFO(node_->get_logger(), "\033[34m巡航模式切换: %s -> %s\033[0m", cruiseModeToString(previous_cruise_mode_).c_str(), cruiseModeToString(current_cruise_mode_).c_str());
            FillGoalsDeque_Cruise();

            previous_cruise_mode_ = current_cruise_mode_;
        }
    }

    /**************************回调函数 end*********************************/
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<hnurm_ul_behavior_trees::PubRobotStatus>("PubRobotStatus");
}