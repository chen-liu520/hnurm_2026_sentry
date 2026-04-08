# linefit_ground_segmentation

地面分割算法的实现,该算法提出于:
```
@inproceedings{himmelsbach2010fast,
  title={Fast segmentation of 3d point clouds for ground vehicles},
  author={Himmelsbach, Michael and Hundelshausen, Felix V and Wuensche, H-J},
  booktitle={Intelligent Vehicles Symposium (IV), 2010 IEEE},
  pages={560--565},
  year={2010},
  organization={IEEE}
}
```
`linefit_ground_segmentation` 包包含地面分割库。
ROS 接口可在 `linefit_ground_segmentation_ros` 中找到。

如果您不使用 ROS,该库可以独立于 ROS 接口进行编译。

## 安装

```bash
colcon build --symlink-install
```

## 启动说明

可以通过执行 `ros2 launch linefit_ground_segmentation_ros segmentation.launch` 来启动地面分割 ROS 节点。
输入和输出话题名称可以在同一文件中指定。

要使用您自己的点云源快速启动和运行,只需:

1. 在 `segmentation.launch` 中将 `input_topic` 参数更改为您的话题。
2. 在 `segmentation_params.yaml` 中调整 `sensor_height` 参数为传感器在机器人上的安装高度(例如 KITTI Velodyne: 1.8m)

## 参数说明

参数设置在 `linefit_ground_segmentation_ros/launch/segmentation_params.yaml` 中

该算法基于您已知传感器距离地面的高度这一假设工作。
因此,**您必须根据机器人规格调整 `sensor_height`**,否则将无法正常工作。

默认参数应该适用于 KITTI 数据集。

### 地面条件
- **sensor_height**  传感器距离地面的高度。
- **max_dist_to_line**  点到直线的最大垂直距离,用于判断是否为地面点。
- **max_slope**  直线的最大斜率。
- **min_slope**  直线的最小斜率。
- **max_fit_error**  在直线拟合中,点允许的最大误差。
- **max_start_height**  新点与估计地面高度之间的最大高度差,用于开始新的直线。
- **long_threshold**  距离阈值,超过此距离后将应用 max_height 条件。
- **max_height**  当直线点之间的距离大于 *long_threshold* 时,它们之间的最大高度差。
- **line_search_angle**  在角度方向上搜索直线的范围。更大的角度有助于填补地面分割中的"空洞"。
- **gravity_aligned_frame**  与重力对齐的坐标系名称(其z轴与重力对齐)。如果指定,输入的点云将被旋转(但不会平移)到该坐标系中。如果留空,将使用传感器坐标系。

### 分割

- **r_min**  分割开始的距离。
- **r_max**  分割结束的距离。
- **n_bins**  径向分箱的数量。
- **n_segments**  角度分段的数量。

### 其他

- **n_threads**  使用的线程数。
- **latch**  在 ROS 节点中锁存输出点云。
- **visualize** 可视化分割结果。**仅用于调试。** 在线运行时请勿设置为 true。
