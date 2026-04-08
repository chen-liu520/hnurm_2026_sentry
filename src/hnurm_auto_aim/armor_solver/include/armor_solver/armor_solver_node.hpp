// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef ARMOR_SOLVER_SOLVER_NODE_HPP_
#define ARMOR_SOLVER_SOLVER_NODE_HPP_

// ros2
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// std
#include <memory>
#include <string>
#include <vector>
// project
#include "armor_solver/armor_solver.hpp"
#include "armor_solver/armor_tracker.hpp"
#include "hnurm_interfaces/msg/armors.hpp"
#include "hnurm_interfaces/msg/measurement.hpp"
#include "hnurm_interfaces/msg/target.hpp"
#include "hnurm_interfaces/srv/set_mode.hpp"
#include "hnurm_utils/heartbeat.hpp"
#include "hnurm_utils/logger/log.hpp"

namespace fyt::auto_aim {
using tf2_filter = tf2_ros::MessageFilter<hnurm_interfaces::msg::Armors>;
class ArmorSolverNode : public rclcpp::Node {
public:
  explicit ArmorSolverNode(const rclcpp::NodeOptions &options);

private:
  void armorsCallback(const hnurm_interfaces::msg::Armors::SharedPtr armors_ptr);

  void initMarkers() noexcept;

  void publishMarkers(const hnurm_interfaces::msg::Target &target_msg,
                      const hnurm_interfaces::msg::VisionSendData &gimbal_cmd) noexcept;


  void setModeCallback(const std::shared_ptr<hnurm_interfaces::srv::SetMode::Request> request,
                       std::shared_ptr<hnurm_interfaces::srv::SetMode::Response> response);
  
  bool debug_mode_;

  // Heartbeat
  HeartBeatPublisher::SharedPtr heartbeat_;

  // The time when the last message was received
  rclcpp::Time last_time_;
  double dt_;

  // Armor tracker
  double s2qx_, s2qy_, s2qz_, s2qyaw_, s2qr_, s2qd_zc_;
  double r_x_, r_y_, r_z_, r_yaw_;
  double lost_time_thres_;
  std::unique_ptr<Tracker> tracker_;

  // Armor Solver
  std::unique_ptr<Solver> solver_;

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<hnurm_interfaces::msg::Armors> armors_sub_;
  hnurm_interfaces::msg::Target armor_target_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // Measurement publisher
  rclcpp::Publisher<hnurm_interfaces::msg::Measurement>::SharedPtr measure_pub_;

  // Publisher
  rclcpp::Publisher<hnurm_interfaces::msg::Target>::SharedPtr target_pub_;
  rclcpp::Publisher<hnurm_interfaces::msg::VisionSendData>::SharedPtr gimbal_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  void timerCallback();
  
  // Enable/Disable Armor Solver
  bool enable_;
  rclcpp::Service<hnurm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker trajectory_marker_;
  visualization_msgs::msg::Marker armors_marker_;
  visualization_msgs::msg::Marker selection_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_SOLVER_SOLVER_NODE_HPP_
