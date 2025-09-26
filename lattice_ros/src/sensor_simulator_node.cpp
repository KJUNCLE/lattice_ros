#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>
#include <random>
#include <chrono>

class SensorSimulator {
public:
    SensorSimulator(ros::NodeHandle& nh) : nh_(nh) {
        // 发布者
        camera_pub_ = nh_.advertise<sensor_msgs::PointCloud>("perception/camera_obstacles", 1);
        radar_pub_ = nh_.advertise<sensor_msgs::PointCloud>("perception/radar_obstacles", 1);
        leader_traj_pub_ = nh_.advertise<nav_msgs::Path>("leader_vehicle/trajectory", 1);
        rtk_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("rtk/current_pose", 1);
        
        // 定时器
        timer_ = nh_.createTimer(ros::Duration(0.1), &SensorSimulator::timerCallback, this);
        
        // 初始化随机数生成器
        generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
        
        ROS_INFO("Sensor simulator started");
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher camera_pub_;
    ros::Publisher radar_pub_;
    ros::Publisher leader_traj_pub_;
    ros::Publisher rtk_pose_pub_;
    ros::Timer timer_;
    
    std::default_random_engine generator_;
    double simulation_time_ = 0.0;
    
    void timerCallback(const ros::TimerEvent& event) {
        simulation_time_ += 0.1;
        
        publishLeaderTrajectory();
        publishRTKPose();
        publishCameraObstacles();
        publishRadarObstacles();
    }
    
    void publishLeaderTrajectory() {
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "map";
        
        // 生成一条简单的前车轨迹（直线 + 转弯）
        for (int i = 0; i < 50; ++i) {
            geometry_msgs::PoseStamped pose;
            pose.header = path.header;
            
            double s = i * 2.0;  // 每2米一个点
            
            if (s < 50.0) {
                // 直线段
                pose.pose.position.x = s;
                pose.pose.position.y = 0.0;
                pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
            } else {
                // 转弯段
                double angle = (s - 50.0) * 0.02;  // 转弯半径50米
                pose.pose.position.x = 50.0 + 50.0 * sin(angle);
                pose.pose.position.y = 50.0 * (1.0 - cos(angle));
                pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(angle/2), cos(angle/2)));
            }
            
            path.poses.push_back(pose);
        }
        
        leader_traj_pub_.publish(path);
    }
    
    void publishRTKPose() {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        
        // 模拟后车位置（跟随前车）
        double leader_x = simulation_time_ * 2.0;  // 假设前车以2m/s行驶
        pose.pose.position.x = leader_x - 10.0;    // 后车距离前车10米
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
        
        rtk_pose_pub_.publish(pose);
    }
    
    void publishCameraObstacles() {
        sensor_msgs::PointCloud cloud;
        cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = "map";
        
        // 模拟一些随机障碍物
        std::uniform_real_distribution<double> x_dist(-10.0, 50.0);
        std::uniform_real_distribution<double> y_dist(-5.0, 5.0);
        std::uniform_real_distribution<double> prob_dist(0.0, 1.0);
        
        for (int i = 0; i < 5; ++i) {
            if (prob_dist(generator_) < 0.3) {  // 30%概率生成障碍物
                geometry_msgs::Point32 point;
                point.x = x_dist(generator_);
                point.y = y_dist(generator_);
                point.z = 0.0;
                cloud.points.push_back(point);
            }
        }
        
        camera_pub_.publish(cloud);
    }
    
    void publishRadarObstacles() {
        sensor_msgs::PointCloud cloud;
        cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = "map";
        
        // 添加速度通道
        cloud.channels.resize(2);
        cloud.channels[0].name = "velocity_x";
        cloud.channels[1].name = "velocity_y";
        
        // 模拟雷达检测的障碍物（包含速度信息）
        std::uniform_real_distribution<double> x_dist(-10.0, 100.0);
        std::uniform_real_distribution<double> y_dist(-8.0, 8.0);
        std::uniform_real_distribution<double> v_dist(-5.0, 5.0);
        std::uniform_real_distribution<double> prob_dist(0.0, 1.0);
        
        for (int i = 0; i < 8; ++i) {
            if (prob_dist(generator_) < 0.4) {  // 40%概率生成障碍物
                geometry_msgs::Point32 point;
                point.x = x_dist(generator_);
                point.y = y_dist(generator_);
                point.z = 0.0;
                cloud.points.push_back(point);
                
                // 添加速度信息
                cloud.channels[0].values.push_back(v_dist(generator_));
                cloud.channels[1].values.push_back(v_dist(generator_));
            }
        }
        
        radar_pub_.publish(cloud);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_simulator_node");
    
    ros::NodeHandle nh;
    
    try {
        SensorSimulator simulator(nh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in sensor simulator: %s", e.what());
        return -1;
    }
    
    return 0;
}