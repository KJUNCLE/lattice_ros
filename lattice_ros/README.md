# Lattice Planner

基于Lattice算法的ROS1局部路径规划器，专为车辆跟随场景设计。

TODO: 

    - [ ] cameraObstaclesCallback 这里的消息格式有问题，需要在确定视觉信息之后进行修改
    - [ ] 雷达障碍物的数据格式需要根据实际情况修改


## 功能特性

- **车辆跟随**: 基于前车RTK轨迹生成跟随路径
- **Lattice采样**: 在Frenet坐标系中生成多条候选轨迹
- **多项式轨迹生成**: 使用五次多项式生成平滑的横向轨迹，四次多项式生成纵向轨迹
- **多传感器融合**: 支持单目相机和4D毫米波雷达的障碍物检测
- **多目标优化**: 综合考虑碰撞、舒适性、效率和参考路径跟踪
- **实时可视化**: 在RViz中显示候选轨迹和最优轨迹

## 系统要求

- ROS Melodic/Noetic
- C++14或更高版本
- Eigen3
- RTK定位系统
- 单目相机感知模块
- 4D毫米波雷达感知模块

## 安装

1. 克隆到catkin工作空间:
```bash
cd ~/catkin_ws/src
git clone <repository_url> lattice_planner
```

2. 安装依赖:
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. 编译:
```bash
catkin_make
source devel/setup.bash
```

## 使用方法

### 基本启动

```bash
roslaunch lattice_planner lattice_planner.launch
```

### 带可视化启动

```bash
roslaunch lattice_planner lattice_planner.launch use_rviz:=true
```

### 带传感器模拟器启动（用于测试）

```bash
roslaunch lattice_planner lattice_planner_with_simulator.launch
```

## 话题接口

### 订阅话题

- `/leader_vehicle/trajectory` (nav_msgs/Path): 前车RTK轨迹
- `/rtk/current_pose` (geometry_msgs/PoseStamped): 当前车辆RTK位置
- `/perception/camera_obstacles` (geometry_msgs/PointCloud): 相机检测的障碍物
- `/perception/radar_obstacles` (geometry_msgs/PointCloud): 4D毫米波雷达检测的障碍物

### 发布话题

- `/lattice_planner/local_path` (nav_msgs/Path): 局部规划路径
- `/cmd_vel` (geometry_msgs/Twist): 速度控制命令
- `/lattice_planner/visualization` (visualization_msgs/MarkerArray): 可视化标记

## 参数配置

### 规划参数

- `planning_horizon`: 规划时域 (默认: 5.0秒)
- `dt`: 时间步长 (默认: 0.2秒)
- `num_lateral_samples`: 横向采样数量 (默认: 5)
- `num_longitudinal_samples`: 纵向采样数量 (默认: 3)

### 跟随参数

- `following_distance`: 跟随距离 (默认: 10.0 m)
- `lateral_offset`: 横向偏移 (默认: 0.0 m)

### 车辆参数

- `max_speed`: 最大速度 (默认: 2.0 m/s)
- `max_acceleration`: 最大加速度 (默认: 1.0 m/s²)
- `max_deceleration`: 最大减速度 (默认: -2.0 m/s²)
- `max_curvature`: 最大曲率 (默认: 1.0 1/m)
- `vehicle_length`: 车辆长度 (默认: 4.5 m)
- `vehicle_width`: 车辆宽度 (默认: 2.0 m)

### 代价函数权重

- `w_collision`: 碰撞代价权重 (默认: 100.0)
- `w_comfort`: 舒适性代价权重 (默认: 10.0)
- `w_efficiency`: 效率代价权重 (默认: 1.0)
- `w_reference`: 参考路径跟踪权重 (默认: 5.0)

## 算法原理

### 车辆跟随

1. **前车轨迹接收**: 通过RTK获取前车的精确轨迹数据
2. **跟随路径生成**: 基于前车轨迹和设定的跟随距离生成参考路径
3. **动态调整**: 根据实时情况调整跟随距离和横向偏移

### Lattice采样

1. **坐标系转换**: 将当前车辆状态从笛卡尔坐标系转换到Frenet坐标系
2. **目标采样**: 在横向和纵向方向生成多个目标点
3. **轨迹生成**: 为每个目标点生成多项式轨迹
4. **轨迹评估**: 使用多目标代价函数评估每条轨迹
5. **最优选择**: 选择代价最小且无碰撞的轨迹

### 碰撞检测

- **车辆模型**: 使用矩形车辆模型，考虑车辆的长度和宽度
- **传感器融合**: 融合单目相机和4D毫米波雷达的障碍物检测数据
- **安全距离**: 可配置的安全边距确保行驶安全
- **多传感器验证**: 通过多传感器数据交叉验证提高检测可靠性

### 代价函数

总代价 = w_collision × 碰撞代价 + w_comfort × 舒适性代价 + w_efficiency × 效率代价 + w_reference × 参考路径代价

## 调试和优化

### 可视化

在RViz中可以看到:
- 灰色线条: 候选轨迹
- 绿色粗线: 选中的最优轨迹
- 红色线条: 局部规划路径

### 参数调优

1. **增加采样数量**: 提高轨迹质量但增加计算量
2. **调整权重**: 根据实际需求平衡不同目标
3. **修改车辆参数**: 确保与实际车辆匹配
4. **优化时域**: 平衡规划距离和实时性

## 故障排除

### 常见问题

1. **无轨迹生成**: 检查全局路径是否正确接收
2. **频繁停车**: 降低碰撞代价权重或增加安全距离
3. **轨迹不平滑**: 增加舒适性代价权重
4. **跟踪偏差**: 增加参考路径跟踪权重

### 日志信息

- `INFO`: 正常运行信息
- `WARN`: 警告信息，通常不影响运行
- `ERROR`: 错误信息，需要检查配置

## 扩展开发

### 添加新的代价函数

在`evaluateTrajectory`函数中添加新的代价项。

### 修改采样策略

在`TrajectoryGenerator`类中修改采样逻辑。

### 集成其他传感器

在`SensorFusion`类中添加新的传感器数据处理方法。

### 传感器数据格式

#### 相机障碍物数据
```
geometry_msgs/PointCloud
- header: 时间戳和坐标系
- points: 障碍物位置点列表
```

#### 4D毫米波雷达数据
```
geometry_msgs/PointCloud
- header: 时间戳和坐标系
- points: 障碍物位置点列表
- channels[0]: velocity_x (径向速度x分量)
- channels[1]: velocity_y (径向速度y分量)
```

#### 前车轨迹数据
```
nav_msgs/Path
- header: 时间戳和坐标系
- poses: 前车轨迹点序列（RTK精确定位）
```

## 许可证

MIT License

## 贡献

欢迎提交Issue和Pull Request来改进这个项目。