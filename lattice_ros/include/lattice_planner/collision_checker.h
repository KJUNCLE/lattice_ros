#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include "trajectory_generator.h"

namespace lattice_planner {

struct VehicleFootprint {
    double length;
    double width;
    std::vector<std::pair<double, double>> corners;  // 车辆四个角点相对于车辆中心的坐标
    
    VehicleFootprint(double l, double w) : length(l), width(w) {
        // 定义车辆四个角点 (前左, 前右, 后右, 后左)
        corners.push_back({l/2, w/2});
        corners.push_back({l/2, -w/2});
        corners.push_back({-l/2, -w/2});
        corners.push_back({-l/2, w/2});
    }
};

class CollisionChecker {
public:
    CollisionChecker(double vehicle_length, double vehicle_width);
    ~CollisionChecker() = default;
    
    // 设置环境信息
    void setCameraObstacles(const std::vector<geometry_msgs::Point>& obstacles);
    void setRadarObstacles(const std::vector<geometry_msgs::Point>& obstacles);
    
    // 碰撞检测
    bool isTrajectoryCollisionFree(const Trajectory& trajectory);
    bool isPointCollisionFree(const TrajectoryPoint& point);
    double calculateCollisionCost(const Trajectory& trajectory);
    
    // 设置参数
    void setSafetyMargin(double margin) { safety_margin_ = margin; }
    void setCameraDetectionRange(double range) { camera_detection_range_ = range; }
    void setRadarDetectionRange(double range) { radar_detection_range_ = range; }
    void setObstacleConfidenceThreshold(double threshold) { obstacle_confidence_threshold_ = threshold; }

private:
    VehicleFootprint vehicle_footprint_;
    std::vector<geometry_msgs::Point> camera_obstacles_;   // 相机检测障碍物
    std::vector<geometry_msgs::Point> radar_obstacles_;    // 4D毫米波雷达障碍物
    
    double safety_margin_;
    bool has_camera_data_;
    bool has_radar_data_;
    
    // 传感器融合参数
    double camera_detection_range_;    // 相机检测范围
    double radar_detection_range_;     // 雷达检测范围
    double obstacle_confidence_threshold_;  // 障碍物置信度阈值
    
    // 碰撞检测辅助函数
    bool isInCameraObstacle(double x, double y);
    bool isInRadarObstacle(double x, double y);
    std::vector<std::pair<double, double>> getVehicleCorners(const TrajectoryPoint& point);
    
    // 传感器融合
    bool isFusedObstacle(double x, double y);
    double getFusedObstacleConfidence(double x, double y);
    
    // 几何计算
    double pointToLineDistance(double px, double py, double x1, double y1, double x2, double y2);
    bool isPointInPolygon(double x, double y, const std::vector<std::pair<double, double>>& polygon);
    
    // 距离场计算
    double calculateDistanceToObstacle(double x, double y);
    double calculateMinDistanceToCamera(double x, double y);
    double calculateMinDistanceToRadar(double x, double y);
};

} // namespace lattice_planner

#endif // COLLISION_CHECKER_H