关于舵轮的配置文件修改：
主要修改点说明（请务必阅读）：

1. AMCL 定位模型 (amcl):
   - 将 robot_model_type 修改为 nav2_amcl::DifferentialMotionModel。
   - 原因：舵轮机器人通常是非全向移动的（不能横移），在定位算法中，使用差速模型比全向模型更准确。

2. MPPI 控制器 (FollowPath) - 核心修改:
   - motion_model: 修改为 "Ackermann"。这是最适合舵轮的运动学模型。
   - vy_std, vy_min, vy_max: 全部设为 0.0。
      - 原因：舵轮无法像麦克纳姆轮那样直接横向移动，必须禁止 Y 轴速度采样。

   - AckermannConstraints:
      - min_turning_r: 设置为 0.45（示例值）。
      - 重要：这是最小转弯半径。您必须根据您的机器人实际机械结构（轴距和最大舵角）来测量并修改这个值。如果设置得太小，机器人可能会卡住或规划出无法执行的路径。

    - critics: 启用了 ConstraintCritic 并建议启用 use_path_orientations，以确保机器人车头朝向与路径一致。

3. DWB 控制器 (FollowPathDWB):

   - 虽然保留了 DWB 作为备用，但将其参数限制为类似差速机器人的行为（Y 轴速度为 0）。
   - 建议：对于舵轮，强烈建议主要使用 MPPI，因为它能更好地处理最小转弯半径约束。

4. 代价地图 (local_costmap / global_costmap):
   - robot_radius: 保留了半径设置，但添加了 footprint（多边形轮廓）的注释示例。
   - 建议：舵轮 AGV 通常是长方形的。为了更安全的避障，建议您测量机器人的长宽，取消 footprint 的注释并填入实际坐标（例如 [[0.3, 0.2], [0.3, -0.2], ...]），同时注释掉 robot_radius。

5. 路径平滑器 (smoother_server):
   - minimum_turning_radius: 设置为 0.45。
     - 原因：确保全局路径规划出来的路线不会有急转弯，符合机器人的机械限制。

6. 速度平滑器 (velocity_smoother):
   - 将 Y 轴的速度和加速度限制全部设为 0。