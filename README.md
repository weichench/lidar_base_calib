# lidar_base_calib

基于地平面的 Livox Mid360 激光雷达外参（roll / pitch / 高度 Z）离线标定工具。

从 rosbag 中读取指定 topic 的点云（支持 `livox_ros_driver2/msg/CustomMsg` 与
`sensor_msgs/msg/PointCloud2` 两种格式），逐帧用 RANSAC 拟合地平面，
由平面法向量与距离得到 roll / pitch / Z，再对多帧结果做统计（中位数 + 均值 ± 标准差）得到最终标定值。

## 原理

地平面在雷达坐标系下满足 `a·x + b·y + c·z + d = 0`，其中 `(a, b, c)` 为单位法向量（强制朝上）：

```
roll  = atan2(b, c)
pitch = atan2(-a, sqrt(b² + c²))
Z     = d            # 传感器离地高度（正值）
```

## 构建

```bash
cd /home/chenchen/ws_excavator_nav
colcon build --packages-select lidar_base_calib
source install/setup.bash
```

## 运行

所有参数集中在 `config/lidar_base_calib.yaml`，程序启动时读取。配置文件路径作为第一个参数传入：

```bash
ros2 run lidar_base_calib lidar_base_calib_node \
  /home/chenchen/ws_excavator_nav/src/lidar_base_calib/config/lidar_base_calib.yaml
```

或通过 launch：

```bash
ros2 launch lidar_base_calib lidar_base_calib.launch.py \
  config_file:=/home/chenchen/ws_excavator_nav/src/lidar_base_calib/config/lidar_base_calib.yaml
```

结果打印到终端，并写入 `output.dir` 指定的目录（不存在则自动创建）：
- `yaml_file`：最终标定结果（中位数 / 均值 / 标准差）
- `frames_file`：各单帧结果，四列依次为 `timestamp_s(绝对时间戳，秒) roll(°) pitch(°) Z(m)`

当 `output.save_pcd` 为 `true` 时，还会额外把点云用最终标定外参转到车体系并保存 `.pcd`：
- 最多保存 `output.max_pcd_frames` 帧，文件名形如 `<topic短名>_base_<N>.pcd`。
- 帧选择**优先静止帧**：程序读取 `imu.topic`，用角速度模长低于阈值（`imu.stationary_angular_threshold`）
  的连续段判定静止；再在静止帧里挑单帧结果最接近最终中位数（`(v-median)/std`）的帧。
- 若 IMU 不可用（topic 留空 / 缺失 / 类型不符 / 无静止段），退化为在所有有效帧里挑
  最接近最终中位数的帧。

## 配置说明

见 `config/lidar_base_calib.yaml` 内注释。常用改动：

- `lidar_topic`：要标定的雷达 topic（bag 内 4 个 Mid360 各一个）。
- `time_range.start_time / end_time`：只使用 bag 中某段时间的数据（相对 bag 起始的秒）。
- `initial_extrinsic`：初始外参；`roll/pitch/z` 会被标定结果覆盖，`yaw/x/y` 保持不变。
- `ground_plane.use_initial_for_ground_filter`：场景较杂乱时可设为 `true`，并给 `initial_extrinsic.z`
  一个近似的安装高度，用于粗筛地面候选点。
- `imu`：可选。配置 `imu.topic` 后，程序会读取 bag 中对应 IMU topic，用静止段的重力方向估计
  roll/pitch，自动写回 `initial_extrinsic`（不再手填 0），使地面粗筛在大倾角安装下更准。
  `imu.topic` 留空则禁用。
