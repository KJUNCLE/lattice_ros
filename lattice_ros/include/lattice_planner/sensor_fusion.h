#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <vector>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Header.h>
#include <ros/time.h>  // 添加时间相关的包含

namespace lattice_planner {

struct ObstacleInfo {
    geometry_msgs::Point position;
    double confidence;
    double velocity_x, velocity_y;  // 4D雷达可提供速度信息
    std::string sensor_type;        // "camera" 或 "radar"
    ros::Time timestamp;
    
    ObstacleInfo() : confidence(0.0), velocity_x(0.0), velocity_y(0.0) {}
};

class SensorFusion {
public:
    SensorFusion();
    ~SensorFusion() = default;
    
    // 设置传感器数据
    void setCameraObstacles(const sensor_msgs::PointCloud& obstacles);
    void setRadarObstacles(const sensor_msgs::PointCloud& obstacles);
    
    // 获取融合后的障碍物信息
    std::vector<ObstacleInfo> getFusedObstacles();
    
    // 配置参数
    void setFusionParameters(double association_threshold, 
                           double camera_weight, 
                           double radar_weight,
                           double max_obstacle_age);
    
    // 预测障碍物运动
    std::vector<ObstacleInfo> predictObstacleMotion(double prediction_time);

private:
    std::vector<ObstacleInfo> camera_obstacles_;
    std::vector<ObstacleInfo> radar_obstacles_;
    std::vector<ObstacleInfo> fused_obstacles_;
    
    // 融合参数
    double association_threshold_;  // 数据关联阈值
    double camera_weight_;         // 相机权重
    double radar_weight_;          // 雷达权重
    double max_obstacle_age_;      // 障碍物最大存活时间
    
    // 数据关联和融合
    void performDataAssociation();
    void fuseAssociatedObstacles();
    void removeOldObstacles();
    
    // 辅助函数
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    ObstacleInfo interpolateObstacles(const ObstacleInfo& obs1, const ObstacleInfo& obs2, double weight);
    bool isObstacleValid(const ObstacleInfo& obstacle);
};

} // namespace lattice_planner

#endif // SENSOR_FUSION_H