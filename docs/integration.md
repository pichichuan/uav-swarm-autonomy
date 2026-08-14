# ROS 2 / PX4 接入边界

核心包不依赖 ROS，因此可重复测试。部署时用以下话题适配：

| 核心对象 | ROS 2 / PX4 输入 | ROS 2 / PX4 输出 |
|---|---|---|
| `DroneState` | PX4 local odometry、VINS/KISS-ICP 位姿 | PX4 Offboard velocity/trajectory setpoint |
| `SemanticObservation` | YOLO 2D 目标 + 深度/LiDAR 关联 | 语义障碍主题、目标任务主题 |
| `OccupancyMap` | 点云/深度图、机间地图增量 | Fast-Planner ESDF/占据图输入 |
| `PeerState` | Zenoh/ROS 2 DDS 机间状态与轨迹 | EGO-Planner-v2 多机轨迹输入 |

安全要求：PX4 仍是最后一层飞控与失效保护；本项目不替代飞控的 geofence、failsafe、RC takeover 或碰撞保护。
