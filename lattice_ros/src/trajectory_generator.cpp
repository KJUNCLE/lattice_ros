#include "lattice_planner/trajectory_generator.h"
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>

namespace lattice_planner {

TrajectoryGenerator::TrajectoryGenerator() {
}

void TrajectoryGenerator::setReferencePath(const nav_msgs::Path& path) {
    reference_path_ = path;
    preprocessReferencePath();
}

std::vector<Trajectory> TrajectoryGenerator::generateTrajectories(
    const geometry_msgs::PoseStamped& current_pose,
    const geometry_msgs::Twist& current_velocity,
    double planning_horizon,
    double dt,
    int num_lateral_samples,
    int num_longitudinal_samples,
    double max_speed,
    double max_acceleration,
    double max_deceleration,
    double max_curvature) {
    
    std::vector<Trajectory> trajectories;
    
    if (reference_path_.poses.empty()) {
        return trajectories;
    }
    
    // 转换到Frenet坐标系
    FrenetState current_frenet = cartesianToFrenet(current_pose, current_velocity);
    
    // 生成采样目标
    auto lateral_targets = generateLateralTargets(num_lateral_samples, 3.0);  // 最大横向偏移3米
    auto longitudinal_targets = generateLongitudinalTargets(
        num_longitudinal_samples, current_velocity.linear.x, max_speed, planning_horizon);
    
    // 为每个目标生成轨迹
    for (double d_target : lateral_targets) {
        for (double s_target : longitudinal_targets) {
            // 生成横向轨迹 (五次多项式)
            auto lateral_coeffs = generateQuinticPolynomial(
                current_frenet.d, current_frenet.d_dot, current_frenet.d_ddot,
                d_target, 0.0, 0.0, planning_horizon);
            
            // 生成纵向轨迹 (四次多项式)
            auto longitudinal_coeffs = generateQuarticPolynomial(
                current_frenet.s, current_frenet.s_dot, current_frenet.s_ddot,
                s_target / planning_horizon, 0.0, planning_horizon);
            
            // 采样轨迹点
            Trajectory trajectory;
            bool valid = true;
            
            for (double t = 0; t <= planning_horizon; t += dt) {
                FrenetState frenet_point;
                
                // 计算横向状态
                double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
                frenet_point.d = lateral_coeffs[0] + lateral_coeffs[1] * t + 
                                lateral_coeffs[2] * t2 + lateral_coeffs[3] * t3 + 
                                lateral_coeffs[4] * t4 + lateral_coeffs[5] * t5;
                frenet_point.d_dot = lateral_coeffs[1] + 2 * lateral_coeffs[2] * t + 
                                    3 * lateral_coeffs[3] * t2 + 4 * lateral_coeffs[4] * t3 + 
                                    5 * lateral_coeffs[5] * t4;
                frenet_point.d_ddot = 2 * lateral_coeffs[2] + 6 * lateral_coeffs[3] * t + 
                                     12 * lateral_coeffs[4] * t2 + 20 * lateral_coeffs[5] * t3;
                
                // 计算纵向状态
                frenet_point.s = longitudinal_coeffs[0] + longitudinal_coeffs[1] * t + 
                                longitudinal_coeffs[2] * t2 + longitudinal_coeffs[3] * t3 + 
                                longitudinal_coeffs[4] * t4;
                frenet_point.s_dot = longitudinal_coeffs[1] + 2 * longitudinal_coeffs[2] * t + 
                                    3 * longitudinal_coeffs[3] * t2 + 4 * longitudinal_coeffs[4] * t3;
                frenet_point.s_ddot = 2 * longitudinal_coeffs[2] + 6 * longitudinal_coeffs[3] * t + 
                                     12 * longitudinal_coeffs[4] * t2;
                
                // 转换到笛卡尔坐标系
                TrajectoryPoint cart_point = frenetToCartesian(frenet_point, current_frenet.s);
                cart_point.t = t;
                
                trajectory.points.push_back(cart_point);
            }
            
            // 验证轨迹
            if (isTrajectoryValid(trajectory, max_speed, max_acceleration, 
                                max_deceleration, max_curvature)) {
                trajectory.is_valid = true;
                trajectories.push_back(trajectory);
            }
        }
    }
    
    return trajectories;
}

FrenetState TrajectoryGenerator::cartesianToFrenet(
    const geometry_msgs::PoseStamped& pose,
    const geometry_msgs::Twist& velocity) {
    
    FrenetState frenet_state;
    
    if (reference_path_.poses.empty()) {
        return frenet_state;
    }
    
    // 找到最近的参考点
    int closest_idx = findClosestPoint(pose);
    
    if (closest_idx < 0 || closest_idx >= reference_path_.poses.size()) {
        return frenet_state;
    }
    
    // 计算纵向位置 (沿参考路径的距离)
    frenet_state.s = reference_s_[closest_idx];
    
    // 计算横向位置 (到参考路径的距离)
    double dx = pose.pose.position.x - reference_path_.poses[closest_idx].pose.position.x;
    double dy = pose.pose.position.y - reference_path_.poses[closest_idx].pose.position.y;
    
    double ref_yaw = tf2::getYaw(reference_path_.poses[closest_idx].pose.orientation);
    frenet_state.d = -dx * sin(ref_yaw) + dy * cos(ref_yaw);
    
    // 计算速度分量 (简化处理)
    double vehicle_yaw = tf2::getYaw(pose.pose.orientation);
    frenet_state.s_dot = velocity.linear.x * cos(vehicle_yaw - ref_yaw);
    frenet_state.d_dot = velocity.linear.x * sin(vehicle_yaw - ref_yaw);
    
    return frenet_state;
}

TrajectoryPoint TrajectoryGenerator::frenetToCartesian(const FrenetState& frenet_state, double s_ref) {
    TrajectoryPoint point;
    
    if (reference_path_.poses.empty()) {
        return point;
    }
    
    // 找到对应的参考点
    int ref_idx = 0;
    double min_diff = std::abs(reference_s_[0] - (s_ref + frenet_state.s));
    
    for (size_t i = 1; i < reference_s_.size(); ++i) {
        double diff = std::abs(reference_s_[i] - (s_ref + frenet_state.s));
        if (diff < min_diff) {
            min_diff = diff;
            ref_idx = i;
        }
    }
    
    // 获取参考点信息
    const auto& ref_pose = reference_path_.poses[ref_idx].pose;
    double ref_x = ref_pose.position.x;
    double ref_y = ref_pose.position.y;
    double ref_yaw = tf2::getYaw(ref_pose.orientation);
    
    // 转换到笛卡尔坐标
    point.x = ref_x - frenet_state.d * sin(ref_yaw);
    point.y = ref_y + frenet_state.d * cos(ref_yaw);
    point.theta = ref_yaw + atan2(frenet_state.d_dot, frenet_state.s_dot);
    point.v = sqrt(frenet_state.s_dot * frenet_state.s_dot + frenet_state.d_dot * frenet_state.d_dot);
    
    // 计算加速度和曲率 (简化)
    point.a = frenet_state.s_ddot;
    if (point.v > 0.1) {
        point.kappa = (frenet_state.s_dot * frenet_state.d_ddot - frenet_state.d_dot * frenet_state.s_ddot) / 
                     pow(point.v, 3);
    } else {
        point.kappa = 0.0;
    }
    
    return point;
}

std::vector<double> TrajectoryGenerator::generateQuinticPolynomial(
    double start_pos, double start_vel, double start_acc,
    double end_pos, double end_vel, double end_acc,
    double T) {
    
    // 五次多项式系数计算
    Eigen::MatrixXd A(6, 6);
    Eigen::VectorXd b(6);
    
    double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
    
    A << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 2, 0, 0, 0,
         1, T, T2, T3, T4, T5,
         0, 1, 2*T, 3*T2, 4*T3, 5*T4,
         0, 0, 2, 6*T, 12*T2, 20*T3;
    
    b << start_pos, start_vel, start_acc, end_pos, end_vel, end_acc;
    
    Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);
    
    return std::vector<double>(coeffs.data(), coeffs.data() + coeffs.size());
}

std::vector<double> TrajectoryGenerator::generateQuarticPolynomial(
    double start_pos, double start_vel, double start_acc,
    double end_vel, double end_acc,
    double T) {
    
    // 四次多项式系数计算
    Eigen::MatrixXd A(5, 5);
    Eigen::VectorXd b(5);
    
    double T2 = T * T, T3 = T2 * T, T4 = T3 * T;
    
    A << 1, 0, 0, 0, 0,
         0, 1, 0, 0, 0,
         0, 0, 2, 0, 0,
         0, 1, 2*T, 3*T2, 4*T3,
         0, 0, 2, 6*T, 12*T2;
    
    b << start_pos, start_vel, start_acc, end_vel, end_acc;
    
    Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);
    
    return std::vector<double>(coeffs.data(), coeffs.data() + coeffs.size());
}

std::vector<double> TrajectoryGenerator::generateLateralTargets(int num_samples, double max_offset) {
    std::vector<double> targets;
    
    if (num_samples <= 1) {
        targets.push_back(0.0);
        return targets;
    }
    
    for (int i = 0; i < num_samples; ++i) {
        double offset = -max_offset + (2.0 * max_offset * i) / (num_samples - 1);
        targets.push_back(offset);
    }
    
    return targets;
}

std::vector<double> TrajectoryGenerator::generateLongitudinalTargets(
    int num_samples, double current_speed, double max_speed, double planning_horizon) {
    
    std::vector<double> targets;
    
    double min_distance = current_speed * planning_horizon;
    double max_distance = max_speed * planning_horizon;
    
    if (num_samples <= 1) {
        targets.push_back((min_distance + max_distance) / 2.0);
        return targets;
    }
    
    for (int i = 0; i < num_samples; ++i) {
        double distance = min_distance + (max_distance - min_distance) * i / (num_samples - 1);
        targets.push_back(distance);
    }
    
    return targets;
}

bool TrajectoryGenerator::isTrajectoryValid(const Trajectory& trajectory, 
                                          double max_speed, double max_acceleration, 
                                          double max_deceleration, double max_curvature) {
    
    for (const auto& point : trajectory.points) {
        // 检查速度限制
        if (point.v > max_speed || point.v < 0) {
            return false;
        }
        
        // 检查加速度限制
        if (point.a > max_acceleration || point.a < max_deceleration) {
            return false;
        }
        
        // 检查曲率限制
        if (std::abs(point.kappa) > max_curvature) {
            return false;
        }
    }
    
    return true;
}

// 计算参考路径上每个点的累积弧长，为后续的 Frenet 坐标系转换提供基础
void TrajectoryGenerator::preprocessReferencePath() {
    reference_s_.clear();
    
    if (reference_path_.poses.empty()) {
        return;
    }
    
    reference_s_.push_back(0.0);
    double cumulative_s = 0.0;
    
    for (size_t i = 1; i < reference_path_.poses.size(); ++i) {
        double distance = calculateDistance(
            reference_path_.poses[i-1].pose.position,
            reference_path_.poses[i].pose.position);
        cumulative_s += distance;
        reference_s_.push_back(cumulative_s);
    }
}

int TrajectoryGenerator::findClosestPoint(const geometry_msgs::PoseStamped& pose) {
    if (reference_path_.poses.empty()) {
        return -1;
    }
    
    int closest_idx = 0;
    double min_distance = calculateDistance(pose.pose.position, 
                                          reference_path_.poses[0].pose.position);
    
    for (size_t i = 1; i < reference_path_.poses.size(); ++i) {
        double distance = calculateDistance(pose.pose.position, 
                                          reference_path_.poses[i].pose.position);
        if (distance < min_distance) {
            min_distance = distance;
            closest_idx = i;
        }
    }
    
    return closest_idx;
}

double TrajectoryGenerator::calculateDistance(const geometry_msgs::Point& p1, 
                                            const geometry_msgs::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

double TrajectoryGenerator::calculateYaw(const geometry_msgs::Point& p1, 
                                       const geometry_msgs::Point& p2) {
    return atan2(p2.y - p1.y, p2.x - p1.x);
}

double TrajectoryGenerator::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Eigen::Vector2d TrajectoryGenerator::rotate2D(const Eigen::Vector2d& vec, double angle) {
    Eigen::Matrix2d rotation;
    rotation << cos(angle), -sin(angle),
                sin(angle), cos(angle);
    return rotation * vec;
}

} // namespace lattice_planner