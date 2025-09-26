#ifndef LATTICE_PLANNER_H
#define LATTICE_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <memory>
#include <Eigen/Dense>

#include "trajectory_generator.h"
#include "collision_checker.h"

namespace lattice_planner {

struct PlannerConfig {
    // 规划参数
    double planning_horizon;      // 规划时域 (s)
    double dt;                   // 时间步长 (s)
    int num_lateral_samples;     // 横向采样数量
    int num_longitudinal_samples; // 纵向采样数量
    
    // 车辆参数
    double max_speed;            // 最大速度 (m/s)
    double max_acceleration;     // 最大加速度 (m/s²)
    double max_deceleration;     // 最大减速度 (m/s²)
    double max_curvature;        // 最大曲率 (1/m)
    double vehicle_length;       // 车辆长度 (m)
    double vehicle_width;        // 车辆宽度 (m)
    
    // 代价函数权重
    double w_collision;          // 碰撞代价权重
    double w_comfort;            // 舒适性代价权重
    double w_efficiency;         // 效率代价权重
    double w_reference;          // 参考路径跟踪权重
};

class LatticePlanner {
public:
    LatticePlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~LatticePlanner() = default;
    
    bool initialize();
    void run();

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    // 订阅者
    ros::Subscriber leader_trajectory_sub_;  // 前车轨迹
    ros::Subscriber current_pose_sub_;       // RTK定位
    ros::Subscriber camera_obstacles_sub_;   // 相机检测的障碍物
    ros::Subscriber radar_obstacles_sub_;    // 4D毫米波雷达障碍物
    
    // 发布者
    ros::Publisher local_path_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher visualization_pub_;
    
    // TF监听器
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // 核心组件
    std::unique_ptr<TrajectoryGenerator> trajectory_generator_;
    std::unique_ptr<CollisionChecker> collision_checker_;
    
    // 配置参数
    PlannerConfig config_;
    
    // 状态变量
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::Twist current_velocity_;
    nav_msgs::Path leader_trajectory_;       // 前车轨迹作为参考路径
    std::vector<geometry_msgs::Point> camera_obstacles_;  // 相机检测障碍物
    std::vector<geometry_msgs::Point> radar_obstacles_;   // 雷达检测障碍物
    
    bool has_leader_trajectory_;
    bool has_current_pose_;
    bool has_camera_data_;
    bool has_radar_data_;
    
    // 跟随参数
    double following_distance_;              // 跟随距离
    double lateral_offset_;                  // 横向偏移
    
    std::string base_frame_;
    std::string map_frame_;
    
    // 回调函数
    void leaderTrajectoryCallback(const nav_msgs::Path::ConstPtr& msg);
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void cameraObstaclesCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    void radarObstaclesCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    
    // 核心规划函数
    bool planLocalPath();
    std::vector<Trajectory> generateCandidateTrajectories();
    double evaluateTrajectory(const Trajectory& trajectory);
    Trajectory selectBestTrajectory(const std::vector<Trajectory>& trajectories);
    
    // 辅助函数
    void loadParameters();
    bool getCurrentVelocity();
    nav_msgs::Path generateFollowingReferencePath();  // 生成跟随参考路径
    void publishLocalPath(const Trajectory& trajectory);
    void publishVisualization(const std::vector<Trajectory>& trajectories, 
                             const Trajectory& best_trajectory);
    
    // 定时器
    ros::Timer planning_timer_;
    void planningTimerCallback(const ros::TimerEvent& event);
};

} // namespace lattice_planner

#endif // LATTICE_PLANNER_H