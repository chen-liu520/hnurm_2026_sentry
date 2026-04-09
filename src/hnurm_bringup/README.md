# hnurm_bringup

`hnurm_bringup` 是一个 ROS 2 包，本说明聚焦于负责 TF 树、里程计对齐与后视目标角度发布的 `tf_transformer_node`。

---

## 包概览

- **核心节点**：`TfTransformer`（位于 `hnurm_bringup/src/tf_transformer_node.cpp`）
- **启动文件**：`launch/tf_transformer.launch.py`
- **参数文件**：`params/extrinsic.yaml`
- **接口依赖**：使用 `hnurm_interfaces` 自定义消息（如 `VisionRecvData`、`ArmorArray`）。

该节点为机器人发布一致的 TF，主要发布视觉自瞄不敏感的部分TF，主要构建`map->odom->base_link->base_footprint`这部分导航敏感的TF。
该节点还将里程计转换到底盘坐标系（原本的里程记输出是odom->base_link，但是导航需要的里程记是odom->base_footprint，这里做一次转换）

---

## TF 树结构

```
map
└── odom
    └── base_link
        └── base_footprint
            ├── back_camera
            ├── lidar_link
            │   └── livox_frame
            └── gimble_link
                └── camera_link
                     └── camera_optical_frame
```

关键说明：
- `base_link` 与 3D 激光雷达（`lidar_link`）共点。
- `base_footprint` 表示底盘在地面上的运动参考系。物理上是滑环轴平面中心，可以近似认为是底盘中心。
- `gimble_link` 是俯仰云台关节，这部分TF在串口节点`uart_node`读取串口数据补偿pitch角度
   - 为什么不补偿yaw？base_link隐含yaw，并且现在是全向底盘，地盘速度解算也是根据云台角度进行解算的，所以我们并不需要一个真正的有正方向的base_footprint。

---

## 话题

### 订阅

| 话题 | 类型 | 作用 |
| --- | --- | --- |
| `/Odometry` | `nav_msgs/msg/Odometry` | FASTLIVO2输出的map->odom里程计。 |
| `vision_recv_data` | `hnurm_interfaces/msg/VisionRecvData` | 提供 IMU roll/pitch/yaw（角度制）和阵营标志。 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 手动设置地图初始位姿（如 RViz 2 中的2D pose estimate）。 |

### 发布

| 话题 | 类型 | 作用 |
| --- | --- | --- |
| `Odometry_transformed` | `nav_msgs/msg/Odometry` | 转换为odom->base_footprint的里程计。 |

---

## 需更新的参数

静态外参位于 `params/extrinsic.yaml`，根据真实传感器布局修改以下区块：

```
lidar_to_base:
  x, y, z, roll, pitch, yaw              # base_footprint 坐标系下的 3D 雷达

joint_to_base:
  x, y, z, roll, pitch, yaw              # base_footprint 坐标系下的云台关节

camera_to_joint:
  x, y, z, roll, pitch, yaw              # joint_link 坐标系下的前视相机

back_camera_to_joint:
  x, y, z, roll, pitch, yaw              # joint_link 坐标系下的后视相机
```

角度单位为弧度。请按照实际测量值填写，确保 TF 对齐准确。

---

## 运行机制

1. **加载参数**：节点启动时读入所有外参并缓存。
2. **发布 TF**：100 ms 定时器广播静态变换和map->odom的动态变换，如果完成一次静态，静态发布会停止
   - `base_link -> base_footprint` 
   - `back_camera -> base_footprint` 
3. **地图初始化**：
   - 在获取阵营前不会发布 `map -> odom`。
   - 接收 `VisionRecvData` 后，根据红/蓝阵营加载预设初始位姿。
   - 若后续收到 `/initialpose`，则以该姿态覆盖预设。
   - **如果重定位节点开始运行，会停止定时器动态发布` map -> odom`，转交给重定位节点进行维护，防止冲突**
4. **里程计转换**：`odom_callback` 将 `odom->base_link` 中的位姿转换到 `odom->base_footprint` 并发布 `Odometry_transformed`。

---

## 配置检查清单

在迁移到新平台时逐项确认：

1. **外参标定**（`params/extrinsic.yaml`）
   - 精确填写四组传感器外参。
   - 可通过 `ros2 run tf2_tools view_frames` 或 RViz TF 显示验证。

2. **视觉数据源**（`vision_recv_data`）
   - 确保发布方提供 roll/pitch/yaw（角度制）与 self_color（1=RED，其余=BLUE）。
   - 若话题命名不同，请在 `tf_transformer_node.cpp` 中同步修改。

3. **里程计输入**（`/Odometry`）
   - 确认上游里程计坐标系符合 `base_footprint` 约定，如不一致需调整 `odom_callback` 中的 TF 查询。

4. **启动文件**
   - 若参数文件非默认 `params/extrinsic.yaml`，请修改 `launch/tf_transformer.launch.py`。

---

## 验证步骤

1. `colcon build --packages-select hnurm_bringup`
2. `ros2 launch hnurm_bringup tf_transformer.launch.py`
3. 使用 RViz 2 查看 TF 树并确认传感器对齐。

---

## 其他说明
- 若比赛场地坐标不同，请修改 `recv_data_callback` 中的初始位姿常量。
```cpp
init_trans_.translation.x = 0.0;
init_trans_.translation.y = 0.0;
init_trans_.translation.z = 0.0;
init_trans_.rotation.x = 0.0;
init_trans_.rotation.y = 0.0;
init_trans_.rotation.z = 0.0;
init_trans_.rotation.w = 1.0;
```
