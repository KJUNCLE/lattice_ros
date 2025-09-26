#include "lattice_planner/lattice_planner.h"
#include <tf2/utils.h>

namespace lattice_planner {

LatticePlanner::LatticePlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), has_leader_trajectory_(false), has_current_pose_(false), 
      has_camera_data_(false), has_radar_data_(false), base_frame_("base_link"), 
      map_frame_("map"), following_distance_(10.0), lateral_offset_(0.0) {
    
    // 初始化TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

bool LatticePlanner::initialize() {
    // 加载参数
    loadParameters();
    
    // 初始化组件
    trajectory_generator_ = std::make_unique<TrajectoryGenerator>();
    collision_checker_ = std::make_unique<CollisionChecker>(
        config_.vehicle_length, config_.vehicle_width);
    
    // 设置订阅者
    // 订阅前车轨迹
    leader_trajectory_sub_ = nh_.subscribe("leader_trajectory", 1, 
        &LatticePlanner::leaderTrajectoryCallback, this);
    // 订阅自车位姿
    current_pose_sub_ = nh_.subscribe("rtk_pose", 1, 
        &LatticePlanner::currentPoseCallback, this);
    // 相机障碍物
    camera_obstacles_sub_ = nh_.subscribe("camera_obstacles", 1, 
        &LatticePlanner::cameraObstaclesCallback, this);
    // 雷达障碍物
    radar_obstacles_sub_ = nh_.subscribe("radar_obstacles", 1, 
        &LatticePlanner::radarObstaclesCallback, this);
    
    // 设置发布者
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("local_path", 1);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    visualization_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "lattice_visualization", 1);
    
    // 设置定时器 10Hz
    planning_timer_ = nh_.createTimer(ros::Duration(0.1), 
        &LatticePlanner::planningTimerCallback, this);
    
    ROS_INFO("Lattice planner initialized successfully");
    return true;
}

void LatticePlanner::loadParameters() {
    // 规划参数
    // 前瞻距离
    pnh_.param("planning_horizon", config_.planning_horizon, 5.0);

    pnh_.param("dt", config_.dt, 0.2);
    // 纵向采样点数
    pnh_.param("num_lateral_samples", config_.num_lateral_samples, 5);
    // 横向采样点数
    pnh_.param("num_longitudinal_samples", config_.num_longitudinal_samples, 3);
    
    // 车辆参数
    pnh_.param("max_speed", config_.max_speed, 2.0);
    pnh_.param("max_acceleration", config_.max_acceleration, 1.0);
    pnh_.param("max_deceleration", config_.max_deceleration, -2.0);
    //转弯半径
    pnh_.param("max_curvature", config_.max_curvature, 1.0);
    pnh_.param("vehicle_length", config_.vehicle_length, 4.5);
    pnh_.param("vehicle_width", config_.vehicle_width, 2.0);
    
    // 代价函数权重
    pnh_.param("w_collision", config_.w_collision, 100.0);
    pnh_.param("w_comfort", config_.w_comfort, 10.0);
    pnh_.param("w_efficiency", config_.w_efficiency, 1.0);
    pnh_.param("w_reference", config_.w_reference, 5.0);
    
    // 跟随参数
    pnh_.param("following_distance", following_distance_, 10.0);
    pnh_.param("lateral_offset", lateral_offset_, 0.0);
    
    // 坐标系
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
}

void LatticePlanner::run() {
    ros::spin();
}

void LatticePlanner::leaderTrajectoryCallback(const nav_msgs::Path::ConstPtr& msg) {
    leader_trajectory_ = *msg;
    has_leader_trajectory_ = true;
    
    // 生成跟随参考路径
    nav_msgs::Path following_path = generateFollowingReferencePath();
    trajectory_generator_->setReferencePath(following_path);
    
    ROS_INFO("Received leader trajectory with %zu points", msg->poses.size());
}

void LatticePlanner::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = *msg;
    has_current_pose_ = true;
}

void LatticePlanner::cameraObstaclesCallback(const sensor_msgs::PointCloud::ConstPtr& msg) {
    camera_obstacles_.clear();
    for (const auto& point32 : msg->points) {
        geometry_msgs::Point point;
        point.x = point32.x;
        point.y = point32.y;
        point.z = point32.z;
        camera_obstacles_.push_back(point);
    }
    has_camera_data_ = true;
    collision_checker_->setCameraObstacles(camera_obstacles_);
}

void LatticePlanner::radarObstaclesCallback(const sensor_msgs::PointCloud::ConstPtr& msg) {
    radar_obstacles_.clear();
    for (const auto& point32 : msg->points) {
        geometry_msgs::Point point;
        point.x = point32.x;
        point.y = point32.y;
        point.z = point32.z;
        radar_obstacles_.push_back(point);
    }
    has_radar_data_ = true;
    collision_checker_->setRadarObstacles(radar_obstacles_);
}

void LatticePlanner::planningTimerCallback(const ros::TimerEvent& event) {
    if (!has_leader_trajectory_ || !has_current_pose_) {
        return;
    }
    
    planLocalPath();
}

bool LatticePlanner::planLocalPath() {
    // 获取当前速度
    if (!getCurrentVelocity()) {
        ROS_WARN("Failed to get current velocity");
        return false;
    }
    
    // 生成候选轨迹
    auto candidate_trajectories = generateCandidateTrajectories();
    if (candidate_trajectories.empty()) {
        ROS_WARN("No valid candidate trajectories generated");
        return false;
    }
    
    // 选择最优轨迹
    auto best_trajectory = selectBestTrajectory(candidate_trajectories);
    if (!best_trajectory.is_valid) {
        ROS_WARN("No valid trajectory found");
        return false;
    }
    
    // 发布结果
    publishLocalPath(best_trajectory);
    publishVisualization(candidate_trajectories, best_trajectory);
    
    return true;
}

std::vector<Trajectory> LatticePlanner::generateCandidateTrajectories() {
    return trajectory_generator_->generateTrajectories(
        current_pose_,
        current_velocity_,
        config_.planning_horizon,
        config_.dt,
        config_.num_lateral_samples,
        config_.num_longitudinal_samples,
        config_.max_speed,
        config_.max_acceleration,
        config_.max_deceleration,
        config_.max_curvature
    );
}

double LatticePlanner::evaluateTrajectory(const Trajectory& trajectory) {
    double cost = 0.0;
    
    // 碰撞代价
    double collision_cost = collision_checker_->calculateCollisionCost(trajectory);
    cost += config_.w_collision * collision_cost;
    
    // 舒适性代价 (加速度和曲率变化)
    double comfort_cost = 0.0;
    for (size_t i = 1; i < trajectory.points.size(); ++i) {
        double da = trajectory.points[i].a - trajectory.points[i-1].a;
        double dk = trajectory.points[i].kappa - trajectory.points[i-1].kappa;
        comfort_cost += da * da + dk * dk;
    }
    cost += config_.w_comfort * comfort_cost;
    
    // 效率代价 (速度偏差)
    double efficiency_cost = 0.0;
    for (const auto& point : trajectory.points) {
        double speed_diff = config_.max_speed - point.v;
        efficiency_cost += speed_diff * speed_diff;
    }
    cost += config_.w_efficiency * efficiency_cost;
    
    return cost;
}

Trajectory LatticePlanner::selectBestTrajectory(const std::vector<Trajectory>& trajectories) {
    Trajectory best_trajectory;
    double min_cost = std::numeric_limits<double>::max();
    
    for (const auto& trajectory : trajectories) {
        if (!trajectory.is_valid) continue;
        
        // 碰撞检测
        if (!collision_checker_->isTrajectoryCollisionFree(trajectory)) {
            continue;
        }
        
        double cost = evaluateTrajectory(trajectory);
        if (cost < min_cost) {
            min_cost = cost;
            best_trajectory = trajectory;
            best_trajectory.cost = cost;
            best_trajectory.is_valid = true;
        }
    }
    
    return best_trajectory;
}

bool LatticePlanner::getCurrentVelocity() {
    try {
        geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
            base_frame_, base_frame_, ros::Time(0), ros::Duration(0.1));
        
        // 简化处理，假设从其他节点获取速度信息
        // 实际应用中可能需要从里程计或其他传感器获取
        current_velocity_.linear.x = 0.0;  // 需要实际实现
        current_velocity_.linear.y = 0.0;
        current_velocity_.angular.z = 0.0;
        
        return true;
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Failed to get current velocity: %s", ex.what());
        return false;
    }
}

void LatticePlanner::publishLocalPath(const Trajectory& trajectory) {
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = map_frame_;
    
    for (const auto& point : trajectory.points) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, point.theta);
        pose.pose.orientation = tf2::toMsg(q);
        
        path_msg.poses.push_back(pose);
    }
    
    local_path_pub_.publish(path_msg);
    
    // 发布控制命令 (简化版本)
    if (!trajectory.points.empty()) {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = trajectory.points[0].v;
        cmd_vel.angular.z = trajectory.points[0].v * trajectory.points[0].kappa;
        cmd_vel_pub_.publish(cmd_vel);
    }
}

void LatticePlanner::publishVisualization(const std::vector<Trajectory>& trajectories, 
                                         const Trajectory& best_trajectory) {
    visualization_msgs::MarkerArray marker_array;
    
    // 可视化候选轨迹
    for (size_t i = 0; i < trajectories.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "candidate_trajectories";
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.scale.x = 0.05;
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
        marker.color.a = 0.5;
        
        for (const auto& point : trajectories[i].points) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(marker);
    }
    
    // 可视化最优轨迹
    if (best_trajectory.is_valid) {
        visualization_msgs::Marker best_marker;
        best_marker.header.frame_id = map_frame_;
        best_marker.header.stamp = ros::Time::now();
        best_marker.ns = "best_trajectory";
        best_marker.id = 0;
        best_marker.type = visualization_msgs::Marker::LINE_STRIP;
        best_marker.action = visualization_msgs::Marker::ADD;
        
        best_marker.scale.x = 0.1;
        best_marker.color.r = 0.0;
        best_marker.color.g = 1.0;
        best_marker.color.b = 0.0;
        best_marker.color.a = 1.0;
        
        for (const auto& point : best_trajectory.points) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            best_marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(best_marker);
    }
    
    visualization_pub_.publish(marker_array);
}

nav_msgs::Path LatticePlanner::generateFollowingReferencePath() {
    nav_msgs::Path following_path;
    following_path.header = leader_trajectory_.header;
    
    if (leader_trajectory_.poses.empty()) {
        return following_path;
    }
    
    // 为每个前车轨迹点生成对应的跟随点
    for (size_t i = 0; i < leader_trajectory_.poses.size(); ++i) {
        const auto& leader_pose = leader_trajectory_.poses[i];
        
        // 计算前车的航向角
        double leader_yaw = tf2::getYaw(leader_pose.pose.orientation);
        
        // 计算跟随位置 (在前车后方following_distance_距离，横向偏移lateral_offset_)
        geometry_msgs::PoseStamped following_pose;
        following_pose.header = leader_pose.header;
        
        // 纵向偏移 (后方)
        following_pose.pose.position.x = leader_pose.pose.position.x - 
            following_distance_ * cos(leader_yaw) - lateral_offset_ * sin(leader_yaw);
        following_pose.pose.position.y = leader_pose.pose.position.y - 
            following_distance_ * sin(leader_yaw) + lateral_offset_ * cos(leader_yaw);
        following_pose.pose.position.z = leader_pose.pose.position.z;
        
        // 保持相同的航向角
        following_pose.pose.orientation = leader_pose.pose.orientation;
        
        following_path.poses.push_back(following_pose);
    }
    
    return following_path;
}

} // namespace lattice_planner