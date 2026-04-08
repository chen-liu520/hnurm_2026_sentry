#include <rclcpp/rclcpp.hpp>
#include <hnurm_interfaces/msg/vision_recv_data.hpp>
#include <hnurm_interfaces/msg/gesture.hpp>
#include <hnurm_interfaces/msg/work_mode.hpp>
#include <hnurm_interfaces/msg/bullet_speed.hpp>
#include <std_msgs/msg/header.hpp>
#include <thread>
#include <atomic>
#include <iostream>
#include <sstream>
#include <mutex>

namespace hnurm_referee_sim
{

    class RefereeSimNode : public rclcpp::Node
    {
    public:
        RefereeSimNode() : Node("referee_sim_node"), current_gesture_(hnurm_interfaces::msg::Gesture::MOVE)
        {
            RCLCPP_INFO(this->get_logger(), "\033[32m[RefereeSimNode] 模拟裁判系统节点启动\033[0m");

            // 声明参数（血量值）
            this->declare_parameter("current_hp", 400.0);
            this->declare_parameter("current_base_hp", 5000.0);
            this->declare_parameter("current_enemy_base_hp", 5000.0);
            this->declare_parameter("current_outpost_hp", 1500.0);
            this->declare_parameter("current_enemy_outpost_hp", 1500.0);
            this->declare_parameter("allow_fire_amount", 500.0);

            // 游戏状态参数
            this->declare_parameter("game_progress", 4.0); // 4=比赛中
            this->declare_parameter("remain_time", 300.0); // 剩余时间（秒），默认7分钟
            this->declare_parameter("center_ctrl", 0.0);   // 0=未被占领, 1=己方, 2=敌方, 3=双方

            // 姿态参数（从话题订阅获取，这里仅作初始值）
            this->declare_parameter("roll", 0.0);
            this->declare_parameter("pitch", 0.0);
            this->declare_parameter("yaw", 0.0);

            // 其他参数（从话题订阅获取，这里仅作初始值）
            this->declare_parameter("self_color", 1);       // 1=RED, 2=BLUE，默认红色
            this->declare_parameter("publish_rate", 500.0); // 发布频率(Hz)，默认500Hz
            this->declare_parameter("control_id", 0.0);
            this->declare_parameter("cmd_x", 0.0);
            this->declare_parameter("cmd_y", 0.0);

            // 初始化从话题接收的数据
            recv_roll_ = this->get_parameter("roll").as_double();
            recv_pitch_ = this->get_parameter("pitch").as_double();
            recv_yaw_ = this->get_parameter("yaw").as_double();
            recv_self_color_ = this->get_parameter("self_color").as_int();
            recv_control_id_ = this->get_parameter("control_id").as_double();

            // 创建发布者
            vision_recv_pub_ = this->create_publisher<hnurm_interfaces::msg::VisionRecvData>(
                "/my_recv_data", 10);

            // 创建订阅者 - 订阅姿态消息
            gesture_sub_ = this->create_subscription<hnurm_interfaces::msg::Gesture>(
                "/decision/gesture",
                rclcpp::SensorDataQoS(),
                std::bind(&RefereeSimNode::gesture_callback, this, std::placeholders::_1));

            // 创建订阅者 - 订阅 vision_recv_data 话题
            vision_recv_sub_ = this->create_subscription<hnurm_interfaces::msg::VisionRecvData>(
                "vision_recv_data",
                rclcpp::SensorDataQoS(),
                std::bind(&RefereeSimNode::vision_recv_callback, this, std::placeholders::_1));

            // 创建定时器发布 VisionRecvData
            double publish_rate = this->get_parameter("publish_rate").as_double();
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
                std::bind(&RefereeSimNode::publish_vision_recv_data, this));

            // 启动输入线程用于手动修改参数
            input_thread_ = std::thread(&RefereeSimNode::input_thread_func, this);

            RCLCPP_INFO(this->get_logger(), "\033[32m[RefereeSimNode] 初始化完成\033[0m");
            RCLCPP_INFO(this->get_logger(), "\033[33m我方颜色初始化为: RED(红色)\033[0m");
            print_help();
        }

        ~RefereeSimNode()
        {
            running_ = false;
            if (input_thread_.joinable())
            {
                input_thread_.join();
            }
        }

    private:
        // 发布者
        rclcpp::Publisher<hnurm_interfaces::msg::VisionRecvData>::SharedPtr vision_recv_pub_;

        // 订阅者
        rclcpp::Subscription<hnurm_interfaces::msg::Gesture>::SharedPtr gesture_sub_;
        rclcpp::Subscription<hnurm_interfaces::msg::VisionRecvData>::SharedPtr vision_recv_sub_;

        // 定时器
        rclcpp::TimerBase::SharedPtr timer_;

        // 当前姿态
        std::atomic<uint8_t> current_gesture_;
        std::mutex gesture_mutex_;

        // 从 vision_recv_data 话题接收的数据
        std::atomic<double> recv_roll_;
        std::atomic<double> recv_pitch_;
        std::atomic<double> recv_yaw_;
        std::atomic<int> recv_self_color_;
        std::atomic<double> recv_control_id_;
        std::mutex recv_data_mutex_;

        // 输入线程控制
        std::atomic<bool> running_{true};
        std::thread input_thread_;

        /**
         * @brief Gesture 消息回调函数
         * @param msg Gesture 消息
         */
        void gesture_callback(const hnurm_interfaces::msg::Gesture::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(gesture_mutex_);
            current_gesture_ = msg->data;
        }

        /**
         * @brief VisionRecvData 消息回调函数 - 从话题接收 roll, pitch, yaw, self_color, control_id
         * @param msg VisionRecvData 消息
         */
        void vision_recv_callback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(recv_data_mutex_);
            recv_roll_ = msg->roll;
            recv_pitch_ = msg->pitch;
            recv_yaw_ = msg->yaw;
            recv_self_color_ = msg->self_color.data;
            recv_control_id_ = msg->control_id;

            // RCLCPP_INFO(this->get_logger(), "\033[36m[VisionRecvData回调] 接收: roll=%.2f, pitch=%.2f, yaw=%.2f, color=%d, control_id=%.1f\033[0m",
            //             recv_roll_.load(), recv_pitch_.load(), recv_yaw_.load(), recv_self_color_.load(), recv_control_id_.load());
        }

        /**
         * @brief 发布 VisionRecvData 消息
         */
        void publish_vision_recv_data()
        {
            auto msg = hnurm_interfaces::msg::VisionRecvData();

            // 填充消息头
            msg.header.stamp = this->now();
            msg.header.frame_id = "referee";

            // 从参数获取血量值（手动输入）
            msg.current_hp = this->get_parameter("current_hp").as_double();
            msg.current_base_hp = this->get_parameter("current_base_hp").as_double();
            msg.current_enemy_base_hp = this->get_parameter("current_enemy_base_hp").as_double();
            msg.current_outpost_hp = this->get_parameter("current_outpost_hp").as_double();
            msg.current_enemy_outpost_hp = this->get_parameter("current_enemy_outpost_hp").as_double();
            msg.allow_fire_amount = this->get_parameter("allow_fire_amount").as_double();

            // 游戏状态（手动输入）
            msg.game_progress = this->get_parameter("game_progress").as_double();
            msg.remain_time = this->get_parameter("remain_time").as_double();
            msg.center_ctrl = this->get_parameter("center_ctrl").as_double();

            // 姿态角度 - 从 vision_recv_data 话题获取
            {
                std::lock_guard<std::mutex> lock(recv_data_mutex_);
                msg.roll = recv_roll_.load();
                msg.pitch = recv_pitch_.load();
                msg.yaw = recv_yaw_.load();
                msg.self_color.data = recv_self_color_.load();
                msg.control_id = recv_control_id_.load();
            }

            // cmd_x, cmd_y 从参数获取（手动输入）
            msg.cmd_x = this->get_parameter("cmd_x").as_double();
            msg.cmd_y = this->get_parameter("cmd_y").as_double();

            // 设置姿态（从订阅的Gesture消息获取）
            {
                std::lock_guard<std::mutex> lock(gesture_mutex_);
                msg.gesture.data = current_gesture_.load();
            }

            // 设置工作模式和弹速（默认值）
            msg.work_mode.data = hnurm_interfaces::msg::WorkMode::AUTO_AIM;
            msg.bullet_speed.data = 25.0; // 默认25m/s

            // 发布消息
            vision_recv_pub_->publish(msg);
        }

        /**
         * @brief 输入线程函数，用于手动修改参数
         */
        void input_thread_func()
        {
            std::string input;

            while (running_ && rclcpp::ok())
            {
                std::cout << "\033[33m[RefereeSim] 输入命令 (help查看帮助): \033[0m";
                std::cout.flush();

                if (!std::getline(std::cin, input))
                {
                    break;
                }

                // 解析命令
                std::istringstream iss(input);
                std::string cmd;
                iss >> cmd;

                if (cmd == "hp")
                {
                    double value;
                    if (iss >> value)
                    {
                        this->set_parameter(rclcpp::Parameter("current_hp", value));
                        RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 当前血量设置为: %.1f\033[0m", value);
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "用法: hp <值>");
                    }
                }
                else if (cmd == "base")
                {
                    double value;
                    if (iss >> value)
                    {
                        this->set_parameter(rclcpp::Parameter("current_base_hp", value));
                        RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 我方基地血量设置为: %.1f\033[0m", value);
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "用法: base <值>");
                    }
                }
                else if (cmd == "ebase")
                {
                    double value;
                    if (iss >> value)
                    {
                        this->set_parameter(rclcpp::Parameter("current_enemy_base_hp", value));
                        RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 敌方基地血量设置为: %.1f\033[0m", value);
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "用法: ebase <值>");
                    }
                }
                else if (cmd == "outpost")
                {
                    double value;
                    if (iss >> value)
                    {
                        this->set_parameter(rclcpp::Parameter("current_outpost_hp", value));
                        RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 我方前哨站血量设置为: %.1f\033[0m", value);
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "用法: outpost <值>");
                    }
                }
                else if (cmd == "eoutpost")
                {
                    double value;
                    if (iss >> value)
                    {
                        this->set_parameter(rclcpp::Parameter("current_enemy_outpost_hp", value));
                        RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 敌方前哨站血量设置为: %.1f\033[0m", value);
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "用法: eoutpost <值>");
                    }
                }
                else if (cmd == "fire")
                {
                    double value;
                    if (iss >> value)
                    {
                        this->set_parameter(rclcpp::Parameter("allow_fire_amount", value));
                        RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 允许发射弹量设置为: %.1f\033[0m", value);
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "用法: fire <值>");
                    }
                }
                else if (cmd == "progress")
                {
                    double value;
                    if (iss >> value)
                    {
                        this->set_parameter(rclcpp::Parameter("game_progress", value));
                        std::string progress_str;
                        if (value == 0.0)
                            progress_str = "未开始比赛";
                        else if (value == 1.0)
                            progress_str = "准备阶段";
                        else if (value == 2.0)
                            progress_str = "十五秒自检阶段";
                        else if (value == 3.0)
                            progress_str = "五秒倒计时";
                        else if (value == 4.0)
                            progress_str = "比赛中";
                        else if (value == 5.0)
                            progress_str = "比赛结算中";
                        else
                            progress_str = "未知状态";
                        RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 游戏进度设置为: %.1f (%s)\033[0m", value, progress_str.c_str());
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "用法: progress <0-5> (0:未开始 1:准备 2:自检 3:倒计时 4:比赛中 5:结算)");
                    }
                }
                else if (cmd == "time" || cmd == "remain_time")
                {
                    double value;
                    if (iss >> value)
                    {
                        this->set_parameter(rclcpp::Parameter("remain_time", value));
                        RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 剩余时间设置为: %.1f 秒 (%.1f 分钟)\033[0m",
                                    value, value / 60.0);
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "用法: time <秒数>");
                    }
                }
                else if (cmd == "center")
                {
                    double value;
                    if (iss >> value)
                    {
                        this->set_parameter(rclcpp::Parameter("center_ctrl", value));
                        std::string center_str;
                        if (value == 0.0)
                            center_str = "未被占领";
                        else if (value == 1.0)
                            center_str = "己方占领";
                        else if (value == 2.0)
                            center_str = "敌方占领";
                        else if (value == 3.0)
                            center_str = "双方占领";
                        else
                            center_str = "未知";
                        RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 中心占领状态设置为: %.1f (%s)\033[0m",
                                    value, center_str.c_str());
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "用法: center <0-3> (0:未占 1:己方 2:敌方 3:双方)");
                    }
                }
                else if (cmd == "cmd")
                {
                    double x, y;
                    if (iss >> x >> y)
                    {
                        this->set_parameter(rclcpp::Parameter("cmd_x", x));
                        this->set_parameter(rclcpp::Parameter("cmd_y", y));
                        RCLCPP_INFO(this->get_logger(), "\033[32m[参数更新] 控制坐标设置为: cmd_x=%.2f, cmd_y=%.2f\033[0m", x, y);
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "用法: cmd <x> <y>");
                    }
                }
                else if (cmd == "show")
                {
                    show_current_status();
                }
                else if (cmd == "help" || cmd == "h")
                {
                    print_help();
                }
                else if (cmd.empty())
                {
                    // 空输入，忽略
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "未知命令: %s, 输入 help 查看帮助", cmd.c_str());
                }
            }
        }

        /**
         * @brief 显示当前状态
         */
        void show_current_status()
        {
            RCLCPP_INFO(this->get_logger(), "\033[35m========== 当前状态 ==========");

            // 血量信息
            RCLCPP_INFO(this->get_logger(), "【血量信息】");
            RCLCPP_INFO(this->get_logger(), "  当前血量: %.1f", this->get_parameter("current_hp").as_double());
            RCLCPP_INFO(this->get_logger(), "  我方基地血量: %.1f", this->get_parameter("current_base_hp").as_double());
            RCLCPP_INFO(this->get_logger(), "  敌方基地血量: %.1f", this->get_parameter("current_enemy_base_hp").as_double());
            RCLCPP_INFO(this->get_logger(), "  我方前哨站血量: %.1f", this->get_parameter("current_outpost_hp").as_double());
            RCLCPP_INFO(this->get_logger(), "  敌方前哨站血量: %.1f", this->get_parameter("current_enemy_outpost_hp").as_double());
            RCLCPP_INFO(this->get_logger(), "  允许发射弹量: %.1f", this->get_parameter("allow_fire_amount").as_double());

            // 游戏状态
            RCLCPP_INFO(this->get_logger(), "【游戏状态】");
            double progress = this->get_parameter("game_progress").as_double();
            std::string progress_str;
            if (progress == 0.0)
                progress_str = "未开始比赛";
            else if (progress == 1.0)
                progress_str = "准备阶段";
            else if (progress == 2.0)
                progress_str = "十五秒自检阶段";
            else if (progress == 3.0)
                progress_str = "五秒倒计时";
            else if (progress == 4.0)
                progress_str = "比赛中";
            else if (progress == 5.0)
                progress_str = "比赛结算中";
            else
                progress_str = "未知状态";
            RCLCPP_INFO(this->get_logger(), "  游戏进度: %.1f (%s)", progress, progress_str.c_str());

            double remain = this->get_parameter("remain_time").as_double();
            RCLCPP_INFO(this->get_logger(), "  剩余时间: %.1f 秒 (%.1f 分钟)", remain, remain / 60.0);

            double center = this->get_parameter("center_ctrl").as_double();
            std::string center_str;
            if (center == 0.0)
                center_str = "未被占领";
            else if (center == 1.0)
                center_str = "己方占领";
            else if (center == 2.0)
                center_str = "敌方占领";
            else if (center == 3.0)
                center_str = "双方占领";
            else
                center_str = "未知";
            RCLCPP_INFO(this->get_logger(), "  中心占领: %.1f (%s)", center, center_str.c_str());

            // 姿态角度（从话题获取）
            RCLCPP_INFO(this->get_logger(), "【姿态角度】(从vision_recv_data话题获取)");
            RCLCPP_INFO(this->get_logger(), "  Roll: %.2f", recv_roll_.load());
            RCLCPP_INFO(this->get_logger(), "  Pitch: %.2f", recv_pitch_.load());
            RCLCPP_INFO(this->get_logger(), "  Yaw: %.2f", recv_yaw_.load());

            // 其他信息（从话题获取）
            RCLCPP_INFO(this->get_logger(), "【其他信息】(从vision_recv_data话题获取)");
            int color = recv_self_color_.load();
            RCLCPP_INFO(this->get_logger(), "  我方颜色: %d (%s)", color,
                        color == 1 ? "RED(红色)" : (color == 2 ? "BLUE(蓝色)" : "UNKNOWN"));
            RCLCPP_INFO(this->get_logger(), "  控制ID: %.1f", recv_control_id_.load());

            // 手动输入的其他参数
            RCLCPP_INFO(this->get_logger(), "【手动输入参数】");
            RCLCPP_INFO(this->get_logger(), "  cmd_x: %.2f", this->get_parameter("cmd_x").as_double());
            RCLCPP_INFO(this->get_logger(), "  cmd_y: %.2f", this->get_parameter("cmd_y").as_double());

            std::string gesture_str;
            switch (current_gesture_.load())
            {
            case hnurm_interfaces::msg::Gesture::ATTACK:
                gesture_str = "ATTACK(攻击)";
                break;
            case hnurm_interfaces::msg::Gesture::MOVE:
                gesture_str = "MOVE(移动)";
                break;
            case hnurm_interfaces::msg::Gesture::DEFEND:
                gesture_str = "DEFEND(防御)";
                break;
            default:
                gesture_str = "UNKNOWN(" + std::to_string(current_gesture_.load()) + ")";
                break;
            }
            RCLCPP_INFO(this->get_logger(), "  当前姿态: %s", gesture_str.c_str());
            RCLCPP_INFO(this->get_logger(), "==============================\033[0m");
        }

        /**
         * @brief 打印帮助信息
         */
        void print_help()
        {
            RCLCPP_INFO(this->get_logger(), "\033[35m========== 命令帮助 ==========");
            RCLCPP_INFO(this->get_logger(), "【血量控制】");
            RCLCPP_INFO(this->get_logger(), "  hp <值>       - 设置当前血量");
            RCLCPP_INFO(this->get_logger(), "  base <值>     - 设置我方基地血量");
            RCLCPP_INFO(this->get_logger(), "  ebase <值>    - 设置敌方基地血量");
            RCLCPP_INFO(this->get_logger(), "  outpost <值>  - 设置我方前哨站血量");
            RCLCPP_INFO(this->get_logger(), "  eoutpost <值> - 设置敌方前哨站血量");
            RCLCPP_INFO(this->get_logger(), "  fire <值>     - 设置允许发射弹量");
            RCLCPP_INFO(this->get_logger(), "【游戏状态】");
            RCLCPP_INFO(this->get_logger(), "  progress <0-5> - 设置游戏进度 (0:未开始 1:准备 2:自检 3:倒计时 4:比赛中 5:结算)");
            RCLCPP_INFO(this->get_logger(), "  time <秒>     - 设置剩余时间");
            RCLCPP_INFO(this->get_logger(), "  center <0-3>  - 设置中心占领状态 (0:未占 1:己方 2:敌方 3:双方)");
            RCLCPP_INFO(this->get_logger(), "【手动输入参数】");
            RCLCPP_INFO(this->get_logger(), "  cmd <x> <y>   - 设置控制坐标 cmd_x, cmd_y");
            RCLCPP_INFO(this->get_logger(), "【其他】");
            RCLCPP_INFO(this->get_logger(), "  show          - 显示当前所有状态");
            RCLCPP_INFO(this->get_logger(), "  help          - 显示此帮助信息");
            RCLCPP_INFO(this->get_logger(), "\033[33m注: roll/pitch/yaw/self_color/control_id 从 vision_recv_data 话题自动获取\033[0m");
            RCLCPP_INFO(this->get_logger(), "==============================\033[0m");
        }
    };

} // namespace hnurm_referee_sim

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<hnurm_referee_sim::RefereeSimNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
