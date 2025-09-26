#include "lattice_planner/sensor_fusion.h"
#include <algorithm>
#include <cmath>

namespace lattice_planner {

SensorFusion::SensorFusion() 
    : association_threshold_(2.0), camera_weight_(0.6), radar_weight_(0.4), max_obstacle_age_(1.0) {
}

void SensorFusion::setCameraObstacles(const sensor_msgs::PointCloud& obstacles) {
    camera_obstacles_.clear();
    
    for (const auto& point32 : obstacles.points) {
        ObstacleInfo obs;
        obs.position.x = point32.x;
        obs.position.y = point32.y;
        obs.position.z = point32.z;
        obs.sensor_type = "camera";
        obs.timestamp = ros::Time::now();
        obs.confidence = 0.8;  // 相机检测置信度
        
        camera_obstacles_.push_back(obs);
    }
}

void SensorFusion::setRadarObstacles(const sensor_msgs::PointCloud& obstacles) {
    radar_obstacles_.clear();
    
    for (size_t i = 0; i < obstacles.points.size(); ++i) {
        ObstacleInfo obs;
        obs.position.x = obstacles.points[i].x;
        obs.position.y = obstacles.points[i].y;
        obs.position.z = obstacles.points[i].z;
        obs.sensor_type = "radar";
        obs.timestamp = ros::Time::now();
        obs.confidence = 0.9;  // 4D雷达检测置信度通常较高
        
        // 如果有速度信息（4D雷达特有）
        if (obstacles.channels.size() >= 2) {
            if (i < obstacles.channels[0].values.size()) {
                obs.velocity_x = obstacles.channels[0].values[i];
            }
            if (i < obstacles.channels[1].values.size()) {
                obs.velocity_y = obstacles.channels[1].values[i];
            }
        }
        
        radar_obstacles_.push_back(obs);
    }
}

std::vector<ObstacleInfo> SensorFusion::getFusedObstacles() {
    // 执行数据关联和融合
    performDataAssociation();
    fuseAssociatedObstacles();
    removeOldObstacles();
    
    return fused_obstacles_;
}

void SensorFusion::setFusionParameters(double association_threshold, 
                                     double camera_weight, 
                                     double radar_weight,
                                     double max_obstacle_age) {
    association_threshold_ = association_threshold;
    camera_weight_ = camera_weight;
    radar_weight_ = radar_weight;
    max_obstacle_age_ = max_obstacle_age;
}

std::vector<ObstacleInfo> SensorFusion::predictObstacleMotion(double prediction_time) {
    std::vector<ObstacleInfo> predicted_obstacles;
    
    for (const auto& obstacle : fused_obstacles_) {
        ObstacleInfo predicted = obstacle;
        
        // 基于速度预测未来位置
        predicted.position.x += obstacle.velocity_x * prediction_time;
        predicted.position.y += obstacle.velocity_y * prediction_time;
        
        // 降低预测置信度
        predicted.confidence *= (1.0 - prediction_time * 0.1);
        
        if (predicted.confidence > 0.3) {  // 只保留高置信度的预测
            predicted_obstacles.push_back(predicted);
        }
    }
    
    return predicted_obstacles;
}

void SensorFusion::performDataAssociation() {
    fused_obstacles_.clear();
    
    std::vector<bool> camera_used(camera_obstacles_.size(), false);
    std::vector<bool> radar_used(radar_obstacles_.size(), false);
    
    // 寻找相机和雷达检测的关联障碍物
    for (size_t i = 0; i < camera_obstacles_.size(); ++i) {
        if (camera_used[i]) continue;
        
        double min_distance = std::numeric_limits<double>::max();
        int best_radar_idx = -1;
        
        for (size_t j = 0; j < radar_obstacles_.size(); ++j) {
            if (radar_used[j]) continue;
            
            double distance = calculateDistance(camera_obstacles_[i].position, 
                                              radar_obstacles_[j].position);
            
            if (distance < association_threshold_ && distance < min_distance) {
                min_distance = distance;
                best_radar_idx = j;
            }
        }
        
        if (best_radar_idx >= 0) {
            // 找到关联，融合两个观测
            ObstacleInfo fused = interpolateObstacles(camera_obstacles_[i], 
                                                    radar_obstacles_[best_radar_idx], 
                                                    camera_weight_);
            fused_obstacles_.push_back(fused);
            
            camera_used[i] = true;
            radar_used[best_radar_idx] = true;
        }
    }
    
    // 添加未关联的相机检测
    for (size_t i = 0; i < camera_obstacles_.size(); ++i) {
        if (!camera_used[i]) {
            fused_obstacles_.push_back(camera_obstacles_[i]);
        }
    }
    
    // 添加未关联的雷达检测
    for (size_t j = 0; j < radar_obstacles_.size(); ++j) {
        if (!radar_used[j]) {
            fused_obstacles_.push_back(radar_obstacles_[j]);
        }
    }
}

void SensorFusion::fuseAssociatedObstacles() {
    // 在performDataAssociation中已经完成了融合
    // 这里可以添加额外的融合逻辑，如卡尔曼滤波等
}

void SensorFusion::removeOldObstacles() {
    ros::Time current_time = ros::Time::now();
    
    fused_obstacles_.erase(
        std::remove_if(fused_obstacles_.begin(), fused_obstacles_.end(),
            [this, current_time](const ObstacleInfo& obs) {
                double age = (current_time - obs.timestamp).toSec();
                return age > max_obstacle_age_;
            }),
        fused_obstacles_.end()
    );
}

double SensorFusion::calculateDistance(const geometry_msgs::Point& p1, 
                                     const geometry_msgs::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

ObstacleInfo SensorFusion::interpolateObstacles(const ObstacleInfo& obs1, 
                                               const ObstacleInfo& obs2, 
                                               double weight) {
    ObstacleInfo fused;
    
    // 位置加权平均
    fused.position.x = weight * obs1.position.x + (1.0 - weight) * obs2.position.x;
    fused.position.y = weight * obs1.position.y + (1.0 - weight) * obs2.position.y;
    fused.position.z = weight * obs1.position.z + (1.0 - weight) * obs2.position.z;
    
    // 速度信息优先使用雷达数据，如果雷达没有速度信息则使用加权平均
    if (obs2.sensor_type == "radar" && (abs(obs2.velocity_x) > 1e-6 || abs(obs2.velocity_y) > 1e-6)) {
        fused.velocity_x = obs2.velocity_x;
        fused.velocity_y = obs2.velocity_y;
    } else if (obs1.sensor_type == "radar" && (abs(obs1.velocity_x) > 1e-6 || abs(obs1.velocity_y) > 1e-6)) {
        fused.velocity_x = obs1.velocity_x;
        fused.velocity_y = obs1.velocity_y;
    } else {
        fused.velocity_x = weight * obs1.velocity_x + (1.0 - weight) * obs2.velocity_x;
        fused.velocity_y = weight * obs1.velocity_y + (1.0 - weight) * obs2.velocity_y;
    }
    
    // 置信度加权平均
    fused.confidence = weight * obs1.confidence + (1.0 - weight) * obs2.confidence;
    
    // 使用较新的时间戳
    fused.timestamp = std::max(obs1.timestamp, obs2.timestamp);
    
    fused.sensor_type = "fused";
    
    return fused;
}

bool SensorFusion::isObstacleValid(const ObstacleInfo& obstacle) {
    // 检查障碍物是否有效
    if (obstacle.confidence < 0.3) return false;
    
    // 检查位置是否合理
    double distance = sqrt(obstacle.position.x * obstacle.position.x + 
                          obstacle.position.y * obstacle.position.y);
    if (distance > 100.0 || distance < 0.5) return false;  // 距离范围检查
    
    // 检查速度是否合理
    double speed = sqrt(obstacle.velocity_x * obstacle.velocity_x + 
                       obstacle.velocity_y * obstacle.velocity_y);
    if (speed > 50.0) return false;  // 速度上限检查
    
    return true;
}

} // namespace lattice_planner