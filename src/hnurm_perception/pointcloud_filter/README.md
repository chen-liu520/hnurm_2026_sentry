# pointcloud_filter 包说明（中文）

本 README 说明 `hnurm_perception/pointcloud_filter` 包的功能、如何构建/运行、各参数说明以及配置要求与注意事项。

## 概述

`pointcloud_filter` 是一个 ROS 2 节点，用于对传入的点云（以及激光扫描）进行简单过滤：
- 移除车体半径（robot body）内的点/激光点（避免自体回波影响）。
- 可按“特殊区域（SpecialArea）”定义的多边形区域过滤（变换到传感器坐标系后生效），用于屏蔽某些环境区域（例如临时障碍、施工区等）。

实现要点：
- 订阅点云与激光消息，基于车辆半径与可选的 SpecialArea 多边形判断是否去除对应点。
- 使用 tf2 获取 map->sensor_frame 的变换，将 map 中定义的 SpecialArea 转换到传感器坐标系。

主要源码位置：
- 节点实现：src/pointcloud_filter_node.cpp
- 头文件：include/pointcloud_filter/pointcloud_filter_node.hpp
- 启动文件：launch/hnurm_pointcloud_filter.launch.py
- 参数：params/default.yaml

## 构建与运行

在工作空间根目录（包含 `src/`）下：

- 构建：
  colcon build --packages-select pointcloud_filter

- 运行（示例使用 launch）：
  ros2 launch pointcloud_filter hnurm_pointcloud_filter.launch.py

如果你使用的是包含该包的更大 workspace，请确保已 source 安装后的 setup 文件，例如：
  source install/setup.bash

## 主题（Topics）与消息类型

默认（可通过参数覆盖）

- 订阅：
  - `lidar_topic` (sensor_msgs/msg/PointCloud2)
    - 默认："/segmentation/obstacle"
    - 输入点云（已去畸变或注册到某一坐标系的点云）
  - `laser_topic` (sensor_msgs/msg/LaserScan)
    - 默认："/laser/scan"
  - `odom_topic` (nav_msgs/msg/Odometry)
    - 默认："/Odometry"，用于获取当前传感器高度（node 中用于更新 current_sensor_height）
  - `filter_topic` (hnurm_interfaces/msg/SpecialArea)
    - 默认："/special_areas"，发布 SpecialArea 消息以启用并设置过滤多边形
  - `/is_in_special_area` (std_msgs/msg/Bool)
    - 目前已弃用（代码中注释掉了旧方法），保留订阅但不使用

- 发布：
  - `output_lidar_topic_` (sensor_msgs/msg/PointCloud2)
    - 默认："/pointcloud"，发布过滤后的点云
  - `output_laser_topic_` (sensor_msgs/msg/LaserScan)
    - 默认："/ld_laserscan"，发布过滤后的激光扫描
  - `/test_laser` (sensor_msgs/msg/LaserScan)
    - 用于调试
  - `/transformed_special_area` (hnurm_interfaces/msg/SpecialArea)
    - 发布已变换到传感器坐标系的 SpecialArea（供调试或其他节点使用）

## 参数（params/default.yaml）与含义

以下以 params/default.yaml 中的默认值为准，也可以在 launch 或 ros2 param set 时覆盖。

参数列表：

- `lidar_topic` (string)
  - 说明：输入点云的 topic 名称。
  - 默认："/segmentation/obstacle"
  - 备注：该点云应为订正/注册后的点云（例如 /cloud_registered），以便坐标系和 TF 对齐。

- `output_lidar_topic_` (string)
  - 说明：输出（过滤后）点云 topic 名称。
  - 默认："/pointcloud"

- `odom_topic` (string)
  - 说明：里程计 topic，用来更新当前传感器（车辆）高度。
  - 默认："/Odometry"
  - 备注：如果没有发布 odom 或消息中不包含 pose.z，则 current_sensor_height 不会被更新；但过滤主要依赖 robot_radius，因此不是必需，但建议配置正确以便将来扩展。

- `sensor_height` (float)
  - 说明：传感器高度（m）。节点会声明该参数但当前实现主要使用 odom 中的 pose.z 更新 current_sensor_height。
  - 默认：0.31

- `base_frame` (string)
  - 说明：车体基准框架名称。
  - 默认："base_footprint"

- `lidar_frame` (string)
  - 说明：发布的点云将使用该 frame_id（filtered_msg.header.frame_id = lidar_frame_）。
  - 默认："odom"（params 中注释提到可选为 /cloud_registered 或去畸变后的点云帧）
  - 重要：确保该 frame 与系统中的 TF 一致（或在 launch 中根据实际情况修改）。

- `laser_frame` (string)
  - 说明：过滤后发布的 LaserScan 使用的 frame_id。
  - 默认："laser_link"

- `laser_topic` (string)
  - 说明：输入激光扫描 topic。
  - 默认："/laser/scan"

- `robot_radius` (float)
  - 说明：车辆半径（m）。节点会将小于该距离的点/激光射线视为车体回波并移除。
  - 默认：0.5
  - 建议：根据实际底盘尺寸适当调大一点以避免自体点残留；但过大可能误删近距离有效障碍物。

- `filter_topic` (string)
  - 说明：SpecialArea 的 topic（hnurm_interfaces/msg/SpecialArea），用于指定需被屏蔽的多边形区域。
  - 默认："/special_areas"
  - 行为：当收到 SpecialArea 消息时，节点会将其视为启用过滤（use_filter = true），并尝试通过 tf 将该区域从 map 坐标系转换到传感器坐标系，然后对点云/激光进行按区域过滤。

## SpecialArea（hnurm_interfaces/msg/SpecialArea）使用说明

- SpecialArea 消息内包含一组二维点（ZoneEndPoint2D），以多边形表示一个区域（通常闭合）。
- 节点会取收到的最新 SpecialArea（current_spa_），并在处理每帧数据时：
  - 使用 tf_buffer_ 查询 map -> sensor_frame（msg header frame） 的变换（lookupTransform），将 SpecialArea 从 map 转换到 sensor frame。
  - 对每个点/激光射线计算其在传感器坐标系下的 (x,y)，并用点-in-polygon（射线法）判断是否位于 SpecialArea 内。
  - 如果在区域内，则将相应的点/射线设置为 +inf（激光）或丢弃（点云）。

注意：SpecialArea 的坐标应以 `map`（或你在系统中定义的全局 frame）为参考，并保证有相应的 TF 可用于 map -> sensor_frame 的查询，否则会报 TF 失败警告并跳过该帧过滤。

## 配置要求与注意事项

1. TF 树完整性
   - 节点在 transform_SpecialArea 时会使用 tf_buffer_->lookupTransform(msg->header.frame_id, "map", ...)
     - 也就是说，它期望存在从 `map` 到 点云/激光消息 header 中 frame 的变换（或反向查询也可以根据 TF 配置调整）。
   - 如果没有可用的 TF（超时或未发布），节点会输出警告并跳过当前帧的基于 SpecialArea 的过滤。确保发布地图/定位相关的 TF（例如 map->odom->base_link->sensor frames）。

2. 传感器 frame 与参数一致性
   - `lidar_frame` 与输入点云的 header.frame_id、以及系统中的 TF 应保持一致。
   - params/default.yaml 中默认将 `lidar_frame` 设置为 "odom"（注释中提到可使用 /cloud_registered）。根据你的点云来源（例如 Livox、Velodyne、点云注册节点）调整该参数。

3. SpecialArea 发布者
   - 确保有节点按 `hnurm_interfaces::msg::SpecialArea` 的格式发布多边形，否则区域过滤不会触发。
   - 节点接收到 SpecialArea 会将 use_filter 置为 true（开启过滤）；目前没有明确的关闭逻辑（可以通过设计发布空区域或添加一个专用的开关 topic 实现开启/关闭）。

4. 激光与点云同步与 QoS
   - 订阅点云与激光使用了 SensorDataQoS（适合传感器流）。
   - 激光输出设置为 reliable QoS（在 node 中对 laser 发布者 QoS 做了 reliable 设置）。根据你网络/系统稳定性调整 QoS 策略。

5. robot_radius 调参
   - robot_radius 决定车体去点半径；若机器人较大建议适当增大。
   - 同时注意，若 radius 过大，会导致近距离真实障碍被误删，影响避障/规划性能。

6. odom_topic 与 sensor_height
   - 节点会订阅 odom 以更新 current_sensor_height，但当前过滤逻辑并未大量使用该值（仅在声明中有参数）。如果对 z 方向进行裁剪或更精细的车体遮挡，需要在该值基础上扩展实现。

## 调试建议与常见问题

- 如果看到日志 `tf failed: ...`：说明 lookupTransform 失败。请检查 TF 是否已发布、frame 名称是否一致、lookupTransform 的参数顺序是否与你的 TF 配置匹配。
- 如果 SpecialArea 过滤没有生效：检查是否正确发布 SpecialArea，topic 名称与参数中的 `filter_topic` 是否一致，确认 TF 可用。
- 如果过滤后点云坐标系不正确：检查 `filtered_msg.header.frame_id`（代码中设置为 `lidar_frame_`），确保下游节点期望的 frame 一致或在参数中修改 `lidar_frame`。
- 如果激光发布异常或丢失：注意激光发布者使用 reliable QoS，确保订阅端 QoS 兼容。

## 如何覆盖参数

- 使用 launch 文件时，可在 launch 中通过 `DeclareLaunchArgument` 或 `ComposableNode` 的 parameter 字段覆盖。
- 运行时也可以用 ros2 param 设置：
  - ros2 param set /PointCloudNode robot_radius 0.6
  - ros2 param set /PointCloudNode lidar_topic /my/pointcloud

（注意节点名是 `PointCloudNode`，请以运行时实际节点名为准）

## 扩展/改进建议

- 增加开关参数或 topic，以便动态开启/关闭 SpecialArea 过滤。
- 支持多个 SpecialArea（当前实现使用最新收到的区域）；可扩展为多区域管理并按优先级处理。
- 在点云中加入 z 高度裁剪（结合 sensor_height）以滤除高度异常点。
- 增加单元测试或 ROS2 试验节点以模拟 SpecialArea 与 TF，从而自动化验证过滤行为。

## 联系与许可

该 README 基于源码与 params 文件自动整理，如需补充实测数据或增加示例 launch 文件，请告知，我可以进一步添加具体示例与调试脚本。

---
README 生成者：自动化助手（基于源码扫描）
