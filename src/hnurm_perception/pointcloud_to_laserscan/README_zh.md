# ROS 2 点云 <-> 激光扫描转换器

这是一个 ROS 2 包，提供组件来转换 `sensor_msgs/msg/PointCloud2` 消息到 `sensor_msgs/msg/LaserScan` 消息，以及反向转换。
这本质上是原始 ROS 1 包的一个移植版本。

## pointcloud\_to\_laserscan::PointCloudToLaserScanNode

这个 ROS 2 组件将 `sensor_msgs/msg/PointCloud2` 消息投影到 `sensor_msgs/msg/LaserScan` 消息中。

### 发布话题

* `scan` (`sensor_msgs/msg/LaserScan`) - 输出的激光扫描。

### 订阅话题

* `cloud_in` (`sensor_msgs/msg/PointCloud2`) - 输入点云。如果没有至少一个订阅者订阅 `scan` 话题，则不会处理任何输入。

### 参数

* `min_height` (double, 默认值: 2.2e-308) - 在点云中采样的最小高度，单位为米。
* `max_height` (double, 默认值: 1.8e+308) - 在点云中采样的最大高度，单位为米。
* `angle_min` (double, 默认值: -π) - 最小扫描角度，单位为弧度。
* `angle_max` (double, 默认值: π) - 最大扫描角度，单位为弧度。
* `angle_increment` (double, 默认值: π/180) - 激光扫描分辨率，单位为弧度/射线。
* `queue_size` (double, 默认值: 检测到的核心数) - 输入点云队列大小。
* `scan_time` (double, 默认值: 1.0/30.0) - 扫描速率，单位为秒。仅用于填充输出激光扫描消息的 scan_time 字段。
* `range_min` (double, 默认值: 0.0) - 返回的最小范围，单位为米。
* `range_max` (double, 默认值: 1.8e+308) - 返回的最大范围，单位为米。
* `target_frame` (str, 默认值: 无) - 如果提供，在转换为激光扫描之前将点云变换到此坐标系。否则，激光扫描将在与输入点云相同的坐标系中生成。
* `transform_tolerance` (double, 默认值: 0.01) - 变换查找的时间容差。仅在提供 `target_frame` 时使用。
* `use_inf` (boolean, 默认值: true) - 如果禁用，将无限范围（无障碍物）报告为 range_max + 1。否则报告为 +inf。

## pointcloud\_to\_laserscan::LaserScanToPointCloudNode

这个 ROS 2 组件将 `sensor_msgs/msg/LaserScan` 消息重新发布为 `sensor_msgs/msg/PointCloud2` 消息。

### 发布话题

* `cloud` (`sensor_msgs/msg/PointCloud2`) - 输出的点云。

### 订阅话题

* `scan_in` (`sensor_msgs/msg/LaserScan`) - 输入激光扫描。如果没有至少一个订阅者订阅 `cloud` 话题，则不会处理任何输入。

### 参数

* `queue_size` (double, 默认值: 检测到的核心数) - 输入激光扫描队列大小。
* `target_frame` (str, 默认值: 无) - 如果提供，在转换为激光扫描之前将点云变换到此坐标系。否则，激光扫描将在与输入点云相同的坐标系中生成。
* `transform_tolerance` (double, 默认值: 0.01) - 变换查找的时间容差。仅在提供 `target_frame` 时使用。