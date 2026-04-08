#include "hnurm_uart/uart_node.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <angles/angles.h>
#include <filesystem>

using namespace std::chrono_literals;

namespace hnurm
{
    void UartNode::run()
    {
        // check if /dev/serial/by-id/ is created
        while (!std::filesystem::exists("/dev/serial/by-id/"))
        {
            RCLCPP_WARN(logger, "Waiting for /dev/serial/by-id/ to be created");
            std::this_thread::sleep_for(1s);
        }

        recv_topic_ = this->declare_parameter("recv_topic", "vision_recv_data");
        send_topic_ = this->declare_parameter("send_topic", "vision_send_data");

        use_control_id_ = this->declare_parameter("use_control_id", false);
        control_id_ = static_cast<float>(this->declare_parameter("control_id", 1.0f));

        serial_codec_ = new SerialCodec(shared_from_this());
        callback_group1_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        callback_group2_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        use_distribution_ = this->declare_parameter("use_distribution", false);
        master_ns_ = this->declare_parameter("master_ns", "main");
        slave_ns_ = this->declare_parameter("slave_ns", "right");
        decision_send_topic_ = this->declare_parameter("decision_send_topic", "/decision/vision_send_data");
        back_target_ = this->declare_parameter("back_target", "/back_target");

        spin_control_topic_ = this->declare_parameter("spin_control_topic", "/decision/spin_control");
        scan_center_angle_topic_ = this->declare_parameter("scan_center_angle_topic", "/decision/scan_center_angle");
        back_target_state_topic_ = this->declare_parameter("back_target_state_topic", "/decision/back_target_state");
        enable_scan_control_topic_ = this->declare_parameter("enable_scan_control_topic", "/decision/enable_180_scan");

        auto twist_topic = this->declare_parameter("twist_topic", "/cmd_vel_remap");

        if (use_distribution_)
        {
            // master_pub_ = create_publisher<hnurm_interfaces::msg::VisionRecvData>(
            //     master_ns_ + "/" + recv_topic_, rclcpp::SensorDataQoS()
            // );
            master_pub_ = create_publisher<hnurm_interfaces::msg::VisionRecvData>(
                recv_topic_, rclcpp::SensorDataQoS());
            // defalt main thread do not use namespace
            slave_pub = create_publisher<hnurm_interfaces::msg::VisionRecvData>(
                slave_ns_ + "/" + recv_topic_, rclcpp::SensorDataQoS());
            RCLCPP_INFO_STREAM(logger, "using distribution");
            RCLCPP_INFO_STREAM(logger, "master ns: " << master_ns_ << ", slave ns: " << slave_ns_);
        }
        else
        {
            master_pub_ = create_publisher<hnurm_interfaces::msg::VisionRecvData>(recv_topic_, rclcpp::SensorDataQoS());
            RCLCPP_INFO_STREAM(logger, "not using distribution");
        }

        auto sub_option = rclcpp::SubscriptionOptions();
        sub_option.callback_group = callback_group2_;
        sub_ = create_subscription<hnurm_interfaces::msg::VisionSendData>(
            send_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&UartNode::sub_callback, shared_from_this(), std::placeholders::_1),
            sub_option);

        decision_sub_ = create_subscription<hnurm_interfaces::msg::VisionSendData>(
            decision_send_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&UartNode::decision_sub_callback, shared_from_this(), std::placeholders::_1),
            sub_option);

        sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
            twist_topic,
            rclcpp::ServicesQoS(),
            std::bind(&UartNode::sub_twist_callback, shared_from_this(), std::placeholders::_1),
            sub_option);

        back_target_sub_ = create_subscription<std_msgs::msg::Float32>(
            back_target_,
            rclcpp::SensorDataQoS(),
            std::bind(&UartNode::back_target_callback, shared_from_this(), std::placeholders::_1),
            sub_option);
        in_special_area_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/is_in_special_area",
            rclcpp::SensorDataQoS(),
            std::bind(&UartNode::special_area_callback, shared_from_this(), std::placeholders::_1),
            sub_option);
        spin_control_sub_ = create_subscription<std_msgs::msg::Float32>(
            spin_control_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&UartNode::spin_control_callback, shared_from_this(), std::placeholders::_1),
            sub_option);
        scan_center_angle_sub_ = create_subscription<std_msgs::msg::Float32>(
            scan_center_angle_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&UartNode::scan_center_angle_callback, shared_from_this(), std::placeholders::_1),
            sub_option);
        back_target_state_sub_ = create_subscription<std_msgs::msg::Bool>(
            back_target_state_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&UartNode::back_target_state_callback, shared_from_this(), std::placeholders::_1),
            sub_option);
        enable_scan_control_sub_ = create_subscription<std_msgs::msg::Bool>(
            enable_scan_control_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&UartNode::enable_scan_control_callback, shared_from_this(), std::placeholders::_1),
            sub_option);

        // tf
        target_frame_ = this->declare_parameter("target_frame", "base_footprint");
        timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());

        // 创建服务客户端：用于更新模式
        set_mode_client_ = create_client<hnurm_interfaces::srv::SetMode>("/armor_detector/set_mode");

        // 创建心跳检测发布者
        heartbeat_publisher_ = fyt::HeartBeatPublisher::create(this);

        uart_thread_ = std::thread([this]()
                                   {
        while(rclcpp::ok() && !stop_flag_)
        {
            timer_callback();
        }
        RCLCPP_WARN(logger, "uart thread exit"); });
    }

    void UartNode::back_target_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (msg->data != 666.0)
        {
            hnurm_interfaces::msg::VisionSendData send_data;
            send_data.vel_x = 2000.0;
            send_data.vel_y = 2000.0;
            send_data.vel_yaw = 2000.0;
            send_data.control_id = 50.0;
            if (serial_codec_->send_data(send_data))
                RCLCPP_INFO(logger, "send back target data");
        }
    }

    void UartNode::decision_sub_callback(hnurm_interfaces::msg::VisionSendData::SharedPtr msg)
    {
        msg->vel_x = 2000.0;
        msg->vel_y = 2000.0;
        msg->vel_yaw = 2000.0;
        msg->spin_ctrl = spin_control_value_;
        std::lock_guard<std::mutex> lock(decision_cache_mutex_);
        cached_gesture_ = msg->gesture.data;
        has_cached_gesture_ = true;
    }

    void UartNode::special_area_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        is_in_special_area = msg->data;
    }

    void UartNode::sub_callback(hnurm_interfaces::msg::VisionSendData::SharedPtr msg)
    {
        msg->vel_x = 2000.0;
        msg->vel_y = 2000.0;
        msg->vel_yaw = 2000.0;
        msg->spin_ctrl = spin_control_value_;
        // msg->spin_ctrl = 4;
        denormalizeAngle(*msg);
        // RCLCPP_INFO(logger, "target state: %d, target type: %d, gesture: %d, pitch: %f, yaw: %f", msg->target_state.data, msg->target_type.data, msg->gesture.data, msg->pitch, msg->yaw);
        if (is_in_special_area)
            return;
        if (serial_codec_->send_data(*msg))
            ;
        // RCLCPP_INFO(logger, "send data");
    }

    void UartNode::sub_twist_callback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        hnurm_interfaces::msg::VisionSendData send_data;

        {
            std::lock_guard<std::mutex> lock(decision_cache_mutex_);
            if (has_cached_gesture_)
            {
                send_data.gesture.data = cached_gesture_;
            }
        }

        send_data.vel_x = static_cast<float>(msg->linear.x);
        send_data.vel_y = static_cast<float>(msg->linear.y);
        send_data.spin_ctrl = spin_control_value_; // 发送小陀螺控制值
        // send_data.control_id = 25.0;  //test
        if (is_in_special_area)
        {
            send_data.control_id = 25.0;
            send_data.vel_yaw = static_cast<float>(msg->linear.z);
        }
        else if (enable_scan_control_)
        {
            send_data.control_id = 25.0;
            send_data.vel_yaw = static_cast<float>(scan_center_angle_);
        }
        if (serial_codec_->send_data(send_data))
            ;
        // RCLCPP_DEBUG(logger, "send fused data, gesture=%u, spin_ctrl=%f", send_data.gesture.data, send_data.spin_ctrl);
        // RCLCPP_INFO(logger, "send data");
    }

    void UartNode::spin_control_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // 保存小陀螺控制值，在 sub_twist_callback 中统一发送给电控
        spin_control_value_ = msg->data;
    }

    void UartNode::enable_scan_control_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // 处理启用扫描控制的数据
        enable_scan_control_ = msg->data;
        // 根据需要将enable_scan_control转换为适当的格式并发送到串口
    }

    void UartNode::scan_center_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // 处理扫描中心角度数据
        scan_center_angle_ = msg->data;
        // 根据需要将scan_center_angle转换为适当的格式并发送到串口
    }

    void UartNode::back_target_state_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // 处理后视目标状态数据
        back_target_state_ = msg->data;
    }
    int counter = 0;
    void UartNode::timer_callback()
    {
        geometry_msgs::msg::TransformStamped static_transform;
        geometry_msgs::msg::TransformStamped transformStamped;
        tf2::Quaternion q;
        hnurm_interfaces::msg::VisionRecvData recv_data;
        if (serial_codec_->try_get_recv_data_for(recv_data))
        {
            // early return
            if (recv_data.self_color.data == hnurm_interfaces::msg::SelfColor::COLOR_NONE)
            {
                RCLCPP_WARN(logger, "self color not set, ignoring this msg");
                return;
            }

            // 坐标系对齐
            current_yaw_ = recv_data.yaw;
            recv_data.yaw = angles::normalize_angle(recv_data.yaw * M_PI / 180.0) * 180.0 / M_PI;

            // 检测模式变化并调用服务
            checkAndUpdateMode(recv_data);

            // if(use_distribution_)
            if (use_distribution_ && use_control_id_)
            {
                // use control id to distribute data
                if (recv_data.control_id < 0)
                {
                    recv_data.header.stamp = now();
                    recv_data.header.frame_id = "serial";
                    master_pub_->publish(recv_data);
                }
                else if (recv_data.control_id > 0)
                {
                    recv_data.header.stamp = now();
                    recv_data.header.frame_id = "serial";
                    slave_pub->publish(recv_data);
                }
                else
                {
                    RCLCPP_WARN_ONCE(logger, "control_id is illegal[0], ignoring further msg");
                }
            }
            else if (use_control_id_ && (control_id_ != recv_data.control_id))
            {
                return;
            }
            else
            {
                recv_data.header.stamp = now();
                recv_data.header.frame_id = "serial";
                if (recv_data.game_progress == 0.0 && recv_data.remain_time == 0.0 && recv_data.center_ctrl == 0.0)
                {
                    recv_data.current_hp = 400.0;
                    recv_data.allow_fire_amount = 750.0;
                }
                master_pub_->publish(recv_data);
                // RCLCPP_INFO(this->get_logger(),"hp:%f ,enrmy outpost hp:%f,allow fire:%f",recv_data.current_hp,recv_data.current_enemy_outpost_hp,recv_data.allow_fire_amount);
                // 发布 tf 变换
                recv_data.pitch = -recv_data.pitch;
                transformStamped.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
                transformStamped.header.frame_id = target_frame_;
                transformStamped.child_frame_id = "gimbal_link";
                transformStamped.transform.translation.x = 0.0;
                transformStamped.transform.translation.y = 0.0;
                transformStamped.transform.translation.z = 0.0;

                // 使用接收到的 roll, pitch, yaw 值
                // pitch 已经在前面取反：recv_data.pitch = -recv_data.pitch;
                auto roll_rad = recv_data.roll * M_PI / 180.0;
                auto pitch_rad = recv_data.pitch * M_PI / 180.0;
                auto yaw_rad = recv_data.yaw * M_PI / 180.0;

                q.setRPY(roll_rad, pitch_rad, yaw_rad);
                transformStamped.transform.rotation = tf2::toMsg(q);
                tf_broadcaster_->sendTransform(transformStamped);
            }
            // RCLCPP_WARN(logger, "publish to master, game_progress=%f, remaining_time=%f", recv_data.game_progress, recv_data.remain_time);
            //        RCLCPP_INFO(logger, "recv data: %f, %f, %f", recv_data.pitch, recv_data.yaw, recv_data.roll);
        }
        else
        {
            if (error_cnt_++ > 100)
            {
                std::thread([this]()
                            { re_launch(); })
                    .detach();
            }
            std::this_thread::sleep_for(10ms);
            RCLCPP_WARN(logger, "no data received from serial for %d times", error_cnt_);
        }
    }

    void UartNode::re_launch()
    {
        stop_flag_ = true;
        uart_thread_.join();
        stop_flag_ = false;

        // check if /dev/serial/by-id/ is created
        while (!std::filesystem::exists("/dev/serial/by-id/"))
        {
            RCLCPP_WARN(logger, "Waiting for /dev/serial/by-id/ to be created");
            std::this_thread::sleep_for(1s);
        }

        error_cnt_ = 0;
        serial_codec_->init_port();
        uart_thread_ = std::thread([this]()
                                   {
        while(rclcpp::ok() && !stop_flag_)
        {
            timer_callback();
        }
        RCLCPP_WARN(logger, "uart thread exit"); });
    }

    void UartNode::checkAndUpdateMode(const hnurm_interfaces::msg::VisionRecvData &recv_data)
    {
        uint8_t self_color = recv_data.self_color.data;
        uint8_t work_mode = recv_data.work_mode.data;
        // RCLCPP_INFO(logger, "Received self_color: %u, work_mode: %u", self_color, work_mode);
        uint8_t my_mode{0};

        if (self_color == hnurm_interfaces::msg::SelfColor::BLUE)
        {
            if (work_mode == hnurm_interfaces::msg::WorkMode::AUTO_AIM)
            {
                my_mode = 0; // 自瞄红
            }
            else if (work_mode == hnurm_interfaces::msg::WorkMode::AUTO_SRUNE)
            {
                my_mode = 2; // 小符红
            }
            else if (work_mode == hnurm_interfaces::msg::WorkMode::AUTO_BRUNE)
            {
                my_mode = 4; // 大符红
            }
            else
            {
                my_mode = 0; // 默认值
            }
        }
        else if (self_color == hnurm_interfaces::msg::SelfColor::RED)
        {
            if (work_mode == hnurm_interfaces::msg::WorkMode::AUTO_AIM)
            {
                my_mode = 1; // 自瞄蓝
            }
            else if (work_mode == hnurm_interfaces::msg::WorkMode::AUTO_SRUNE)
            {
                my_mode = 3; // 小符蓝
            }
            else if (work_mode == hnurm_interfaces::msg::WorkMode::AUTO_BRUNE)
            {
                my_mode = 5; // 大符蓝
            }
            else
            {
                my_mode = 1; // 默认值
            }
        }
        else
        {
            my_mode = 0; // 默认值
        }

        // 检测模式变化并调用服务
        if (my_mode != last_mode_)
        {
            // 等待服务可用
            if (!set_mode_client_->wait_for_service(1s))
            {
                RCLCPP_WARN(logger, "Service /armor_detector/set_mode not available, waiting...");
                return;
            }

            auto request = std::make_shared<hnurm_interfaces::srv::SetMode::Request>();
            request->mode = my_mode;

            set_mode_client_->async_send_request(
                request,
                [this, mode = my_mode](
                    rclcpp::Client<hnurm_interfaces::srv::SetMode>::SharedFuture future)
                {
                    try
                    {
                        auto response = future.get();
                        if (response->success)
                        {
                            this->last_mode_ = mode;
                            RCLCPP_INFO(this->logger, "Updated mode to %d: %s", mode, response->message.c_str());
                        }
                        else
                        {
                            RCLCPP_ERROR(
                                this->logger, "Failed to update mode: %s (will retry)", response->message.c_str());
                        }
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_ERROR(this->logger, "Service call exception: %s (will retry)", e.what());
                    }
                });
        }
    }

    void UartNode::denormalizeAngle(hnurm_interfaces::msg::VisionSendData &send_data)
    {
        double send_yaw = send_data.yaw;
        double recv_yaw = angles::normalize_angle(current_yaw_ * M_PI / 180.0) * 180.0 / M_PI;

        // 计算圈数：current_yaw_ 可能超过 ±180，需要计算它在哪个360度区间
        int circle_count = std::floor((current_yaw_ + 180.0) / 360.0);

        // 计算归一化角度差值
        double diff = send_yaw - recv_yaw;

        // 调整目标角度到正确的圈数
        if (std::abs(diff) <= 180.0)
        {
            send_data.yaw = send_yaw + circle_count * 360.0;
        }
        else if (diff > 0)
        {
            send_data.yaw = send_yaw + (circle_count - 1) * 360.0;
        }
        else
        {
            send_data.yaw = send_yaw + (circle_count + 1) * 360.0;
        }

        // RCLCPP_INFO(this->logger, "send_yaw: %f, recv_yaw: %f, send_data.yaw: %f, current_yaw_: %f", send_yaw, recv_yaw, send_data.yaw,  current_yaw_);
    }

} // namespace hnurm
