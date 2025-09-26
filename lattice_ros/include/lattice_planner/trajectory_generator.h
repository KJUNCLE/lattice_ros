#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

namespace lattice_planner {

struct TrajectoryPoint {
    double x, y, theta;          // 位置和航向角
    double v, a;                 // 速度和加速度
    double kappa;                // 曲率
    double t;                    // 时间戳
    
    TrajectoryPoint() : x(0), y(0), theta(0), v(0), a(0), kappa(0), t(0) {}
    TrajectoryPoint(double x_, double y_, double theta_, double v_, double a_, double kappa_, double t_)
        : x(x_), y(y_), theta(theta_), v(v_), a(a_), kappa(kappa_), t(t_) {}
};

struct Trajectory {
    std::vector<TrajectoryPoint> points;
    double cost;
    bool is_valid;
    
    Trajectory() : cost(std::numeric_limits<double>::max()), is_valid(false) {}
};

struct FrenetState {
    double s, s_dot, s_ddot;     // 纵向位置、速度、加速度
    double d, d_dot, d_ddot;     // 横向位置、速度、加速度
    
    FrenetState() : s(0), s_dot(0), s_ddot(0), d(0), d_dot(0), d_ddot(0) {}
    FrenetState(double s_, double s_dot_, double s_ddot_, double d_, double d_dot_, double d_ddot_)
        : s(s_), s_dot(s_dot_), s_ddot(s_ddot_), d(d_), d_dot(d_dot_), d_ddot(d_ddot_) {}
};

class TrajectoryGenerator {
public:
    TrajectoryGenerator();
    ~TrajectoryGenerator() = default;
    
    // 设置参考路径
    void setReferencePath(const nav_msgs::Path& path);
    
    // 生成候选轨迹
    std::vector<Trajectory> generateTrajectories(
        const geometry_msgs::PoseStamped& current_pose,
        const geometry_msgs::Twist& current_velocity,
        double planning_horizon,
        double dt,
        int num_lateral_samples,
        int num_longitudinal_samples,
        double max_speed,
        double max_acceleration,
        double max_deceleration,
        double max_curvature
    );

private:
    nav_msgs::Path reference_path_;
    std::vector<double> reference_s_;  // 参考路径的累积弧长
    
    // Frenet坐标系转换
    FrenetState cartesianToFrenet(const geometry_msgs::PoseStamped& pose,
                                  const geometry_msgs::Twist& velocity);
    TrajectoryPoint frenetToCartesian(const FrenetState& frenet_state, double s_ref);
    
    // 五次多项式轨迹生成
    std::vector<double> generateQuinticPolynomial(
        double start_pos, double start_vel, double start_acc,
        double end_pos, double end_vel, double end_acc,
        double T
    );
    
    // 四次多项式轨迹生成
    std::vector<double> generateQuarticPolynomial(
        double start_pos, double start_vel, double start_acc,
        double end_vel, double end_acc,
        double T
    );
    
    // 轨迹采样
    std::vector<double> generateLateralTargets(int num_samples, double max_offset);
    std::vector<double> generateLongitudinalTargets(int num_samples, double current_speed,
                                                   double max_speed, double planning_horizon);
    
    // 轨迹验证
    bool isTrajectoryValid(const Trajectory& trajectory, double max_speed,
                          double max_acceleration, double max_deceleration,
                          double max_curvature);
    
    // 参考路径处理
    void preprocessReferencePath();
    int findClosestPoint(const geometry_msgs::PoseStamped& pose);
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    double calculateYaw(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    
    // 数学工具函数
    double normalizeAngle(double angle);
    Eigen::Vector2d rotate2D(const Eigen::Vector2d& vec, double angle);
};

} // namespace lattice_planner

#endif // TRAJECTORY_GENERATOR_H