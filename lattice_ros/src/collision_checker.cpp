#include "lattice_planner/collision_checker.h"
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>

namespace lattice_planner {

CollisionChecker::CollisionChecker(double vehicle_length, double vehicle_width)
    : vehicle_footprint_(vehicle_length, vehicle_width), safety_margin_(0.2),
      has_camera_data_(false), has_radar_data_(false), 
      camera_detection_range_(50.0), radar_detection_range_(100.0),
      obstacle_confidence_threshold_(0.5) {
}

void CollisionChecker::setCameraObstacles(const std::vector<geometry_msgs::Point>& obstacles) {
    camera_obstacles_ = obstacles;
    has_camera_data_ = true;
}

void CollisionChecker::setRadarObstacles(const std::vector<geometry_msgs::Point>& obstacles) {
    radar_obstacles_ = obstacles;
    has_radar_data_ = true;
}

bool CollisionChecker::isTrajectoryCollisionFree(const Trajectory& trajectory) {
    for (const auto& point : trajectory.points) {
        if (!isPointCollisionFree(point)) {
            return false;
        }
    }
    return true;
}

bool CollisionChecker::isPointCollisionFree(const TrajectoryPoint& point) {
    // 获取车辆四个角点
    auto corners = getVehicleCorners(point);
    
    // 检查每个角点是否与障碍物碰撞
    for (const auto& corner : corners) {
        if (isFusedObstacle(corner.first, corner.second)) {
            return false;
        }
    }
    
    // 检查车辆中心点
    if (isFusedObstacle(point.x, point.y)) {
        return false;
    }
    
    return true;
}

double CollisionChecker::calculateCollisionCost(const Trajectory& trajectory) {
    double total_cost = 0.0;
    
    for (const auto& point : trajectory.points) {
        // 计算到最近障碍物的距离
        double min_distance = calculateDistanceToObstacle(point.x, point.y);
        
        // 如果距离小于安全距离，增加代价
        if (min_distance < safety_margin_) {
            if (min_distance <= 0) {
                return std::numeric_limits<double>::max();  // 碰撞
            }
            
            // 距离越近，代价越高
            double cost = (safety_margin_ - min_distance) / safety_margin_;
            total_cost += cost * cost;
        }
    }
    
    return total_cost;
}

bool CollisionChecker::isInCameraObstacle(double x, double y) {
    if (!has_camera_data_) {
        return false;
    }
    
    // 检查点是否在相机检测的障碍物附近
    for (const auto& obstacle : camera_obstacles_) {
        double distance = sqrt(pow(x - obstacle.x, 2) + pow(y - obstacle.y, 2));
        if (distance < safety_margin_) {
            return true;
        }
    }
    
    return false;
}

bool CollisionChecker::isInRadarObstacle(double x, double y) {
    if (!has_radar_data_) {
        return false;
    }
    
    // 检查点是否在4D毫米波雷达检测的障碍物附近
    for (const auto& obstacle : radar_obstacles_) {
        double distance = sqrt(pow(x - obstacle.x, 2) + pow(y - obstacle.y, 2));
        if (distance < safety_margin_) {
            return true;
        }
    }
    
    return false;
}

bool CollisionChecker::isFusedObstacle(double x, double y) {
    // 传感器融合：相机和雷达数据的综合判断
    bool camera_detection = isInCameraObstacle(x, y);
    bool radar_detection = isInRadarObstacle(x, y);
    
    // 如果任一传感器检测到障碍物，则认为存在障碍物
    // 可以根据实际需求调整融合策略
    return camera_detection || radar_detection;
}

double CollisionChecker::getFusedObstacleConfidence(double x, double y) {
    double camera_confidence = 0.0;
    double radar_confidence = 0.0;
    
    // 计算相机检测置信度
    if (has_camera_data_) {
        double min_camera_dist = calculateMinDistanceToCamera(x, y);
        if (min_camera_dist < camera_detection_range_) {
            camera_confidence = 1.0 - (min_camera_dist / camera_detection_range_);
        }
    }
    
    // 计算雷达检测置信度
    if (has_radar_data_) {
        double min_radar_dist = calculateMinDistanceToRadar(x, y);
        if (min_radar_dist < radar_detection_range_) {
            radar_confidence = 1.0 - (min_radar_dist / radar_detection_range_);
        }
    }
    
    // 融合置信度 (取最大值或加权平均)
    return std::max(camera_confidence, radar_confidence);
}

std::vector<std::pair<double, double>> CollisionChecker::getVehicleCorners(const TrajectoryPoint& point) {
    std::vector<std::pair<double, double>> corners;
    
    double cos_theta = cos(point.theta);
    double sin_theta = sin(point.theta);
    
    for (const auto& corner : vehicle_footprint_.corners) {
        double local_x = corner.first;
        double local_y = corner.second;
        
        // 旋转并平移到全局坐标系
        double global_x = point.x + local_x * cos_theta - local_y * sin_theta;
        double global_y = point.y + local_x * sin_theta + local_y * cos_theta;
        
        corners.push_back({global_x, global_y});
    }
    
    return corners;
}



double CollisionChecker::pointToLineDistance(double px, double py, 
                                           double x1, double y1, double x2, double y2) {
    double A = px - x1;
    double B = py - y1;
    double C = x2 - x1;
    double D = y2 - y1;
    
    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    
    if (len_sq < 1e-6) {
        return sqrt(A * A + B * B);
    }
    
    double param = dot / len_sq;
    
    double xx, yy;
    if (param < 0) {
        xx = x1;
        yy = y1;
    } else if (param > 1) {
        xx = x2;
        yy = y2;
    } else {
        xx = x1 + param * C;
        yy = y1 + param * D;
    }
    
    double dx = px - xx;
    double dy = py - yy;
    return sqrt(dx * dx + dy * dy);
}

bool CollisionChecker::isPointInPolygon(double x, double y, 
                                       const std::vector<std::pair<double, double>>& polygon) {
    int n = polygon.size();
    bool inside = false;
    
    for (int i = 0, j = n - 1; i < n; j = i++) {
        double xi = polygon[i].first, yi = polygon[i].second;
        double xj = polygon[j].first, yj = polygon[j].second;
        
        if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
    }
    
    return inside;
}

double CollisionChecker::calculateDistanceToObstacle(double x, double y) {
    double min_distance = std::numeric_limits<double>::max();
    
    // 检查相机检测的障碍物
    double camera_distance = calculateMinDistanceToCamera(x, y);
    min_distance = std::min(min_distance, camera_distance);
    
    // 检查雷达检测的障碍物
    double radar_distance = calculateMinDistanceToRadar(x, y);
    min_distance = std::min(min_distance, radar_distance);
    
    return min_distance;
}

double CollisionChecker::calculateMinDistanceToCamera(double x, double y) {
    if (!has_camera_data_ || camera_obstacles_.empty()) {
        return std::numeric_limits<double>::max();
    }
    
    double min_distance = std::numeric_limits<double>::max();
    
    for (const auto& obstacle : camera_obstacles_) {
        double distance = sqrt(pow(x - obstacle.x, 2) + pow(y - obstacle.y, 2));
        min_distance = std::min(min_distance, distance);
    }
    
    return min_distance;
}

double CollisionChecker::calculateMinDistanceToRadar(double x, double y) {
    if (!has_radar_data_ || radar_obstacles_.empty()) {
        return std::numeric_limits<double>::max();
    }
    
    double min_distance = std::numeric_limits<double>::max();
    
    for (const auto& obstacle : radar_obstacles_) {
        double distance = sqrt(pow(x - obstacle.x, 2) + pow(y - obstacle.y, 2));
        min_distance = std::min(min_distance, distance);
    }
    
    return min_distance;
}

} // namespace lattice_planner