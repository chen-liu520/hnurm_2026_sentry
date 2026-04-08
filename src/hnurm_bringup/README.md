# hnurm_bringup

`hnurm_bringup` 是一个 ROS 2 包，本说明聚焦于负责 TF 树、里程计对齐与后视目标角度发布的 `tf_transformer_node`。

---

## 包概览

- **核心节点**：`TfTransformer`（位于 `hnurm_bringup/src/tf_transformer_node.cpp`）
- **启动文件**：`launch/tf_transformer.launch.py`
- **参数文件**：`params/extrinsic.yaml`
- **接口依赖**：使用 `hnurm_interfaces` 自定义消息（如 `VisionRecvData`、`ArmorArray`）。

该节点为机器人发布一致的 TF 树，将里程计转换到底盘坐标系，将速度指令旋转到正确参考系，并依据后视相机装甲板检测结果发布目标角度。

---

## TF 树结构

```
map
└── odom
    └── base_link
        └── base_footprint
            ├── laser_link
            ├── lidar_link
            │   └── livox_frame
            └── joint_link
                ├── imu_link
                ├── camera_link
                └── back_camera
```

关键说明：
- `base_link` 与 3D 激光雷达（`lidar_link`）共点。
- `base_footprint` 表示底盘在地面上的运动参考系。
- `joint_link` 是俯仰云台关节，每次定时器触发都会根据 IMU pitch 进行补偿。
- 仅在获取到机器人阵营或收到初始位姿后，才会发布 `map -> odom`。

---

## 话题

### 订阅

| 话题 | 类型 | 作用 |
| --- | --- | --- |
| `/Odometry` | `nav_msgs/msg/Odometry` | 底盘坐标系下的原始里程计数据。 |
| `vision_recv_data` | `hnurm_interfaces/msg/VisionRecvData` | 提供 IMU roll/pitch/yaw（角度制）和阵营标志。 |
| `/back_camera/back_armor` | `hnurm_interfaces/msg/ArmorArray` | 后视相机装甲板检测结果。 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 以 `base_footprint_fake` 表示的速度指令。 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 手动设置地图初始位姿（如 RViz 2 中的定位）。 |

### 发布

| 话题 | 类型 | 作用 |
| --- | --- | --- |
| `Odometry_transformed` | `nav_msgs/msg/Odometry` | 转换到 `base_link` / `base_footprint` 的里程计。 |
| `/cmd_vel_fake` | `geometry_msgs/msg/Twist` | 旋转到真实 `base_footprint` 的速度指令。 |
| `back_target` | `std_msgs/msg/Float32` | 后视装甲板偏角（角度制）。 |
| TF | `geometry_msgs/msg/TransformStamped` | 发布上述动态与静态坐标变换。 |

---

## 需更新的参数

静态外参位于 `params/extrinsic.yaml`，根据真实传感器布局修改以下区块：

```
laser_to_base:
  x, y, z, roll, pitch, yaw              # base_footprint 坐标系下的 2D 雷达(m)

lidar_to_base:
  x, y, z, roll, pitch, yaw              # base_footprint 坐标系下的 3D 雷达

joint_to_base:
  x, y, z, roll, pitch, yaw              # base_footprint 坐标系下的云台关节

camera_to_joint:
  x, y, z, roll, pitch, yaw              # joint_link 坐标系下的前视相机

imu_to_joint:
  x, y, z, roll, pitch, yaw              # joint_link 坐标系下的 IMU

back_camera_to_joint:
  x, y, z, roll, pitch, yaw              # joint_link 坐标系下的后视相机
```

角度单位为弧度。请按照实际测量值填写，确保 TF 对齐准确。

---

## 运行机制

1. **加载参数**：节点启动时读入所有外参并缓存。
2. **发布 TF**：10 ms 定时器同时广播静态与动态变换，包括：
   - `odom -> hip_imu_footprint` 使用串口数据中的 roll/pitch/yaw。
   - `base_footprint -> joint_link` 根据当前 pitch 动态补偿。
3. **地图初始化**：
   - 在获取阵营前不会发布 `map -> odom`。
   - 接收 `VisionRecvData` 后，根据红/蓝阵营加载预设初始位姿。
   - 若后续收到 `/initialpose`，则以该姿态覆盖预设。
4. **里程计转换**：`odom_callback` 将 `/Odometry` 中的位姿从 `base_footprint` 转换到 `base_link` 并发布 `Odometry_transformed`。
5. **速度指令旋转**：`twist_callback` 将 `/cmd_vel` 转换到真实的 `base_footprint`，输出 `/cmd_vel_fake`。
6. **后视目标角度**：`back_camera_callback` 取第一块装甲的中心点，计算偏角（角度制）并发布到 `back_target`。

---

## 配置检查清单

在迁移到新平台时逐项确认：

1. **外参标定**（`params/extrinsic.yaml`）
   - 精确填写六组传感器外参。
   - 可通过 `ros2 run tf2_tools view_frames` 或 RViz TF 显示验证。

2. **视觉数据源**（`vision_recv_data`）
   - 确保发布方提供 roll/pitch/yaw（角度制）与 self_color（1=RED，其余=BLUE）。
   - 若话题命名不同，请在 `tf_transformer_node.cpp` 中同步修改。

3. **里程计输入**（`/Odometry`）
   - 确认上游里程计坐标系符合 `base_footprint` 约定，如不一致需调整 `odom_callback` 中的 TF 查询。

4. **后视装甲识别**（`/back_camera/back_armor`）
   - 当前逻辑仅使用第一块装甲，如需多目标策略请自行扩展。

5. **速度坐标系**
   - 若不使用 `base_footprint_fake`，可在 `odom_callback` 中恢复 TF 发布或调整 `twist_callback` 的查询。

6. **启动文件**
   - 若参数文件非默认 `params/extrinsic.yaml`，请修改 `launch/tf_transformer.launch.py`。

---

## 验证步骤

1. `colcon build --packages-select hnurm_bringup`
2. `ros2 launch hnurm_bringup tf_transformer.launch.py`
3. 使用 RViz 2 查看 TF 树并确认传感器对齐。
4. 发布测试用 `VisionRecvData`，验证地图初始化和 IMU 姿态。
5. 发布示例 `Odometry`、`ArmorArray`，检查输出话题是否符合预期。

---

## 其他说明

- `tf_transformer_node` 运行在 `MultiThreadedExecutor` 中，回调并行执行，注意下游处理的线程安全。
- 重复复用静态 TF 广播器属于正常用法，每个子坐标系唯一即可。
- 若比赛场地坐标不同，请修改 `recv_data_callback` 中的初始位姿常量。
