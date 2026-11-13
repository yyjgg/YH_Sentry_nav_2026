#ifndef PATH_PLANNING_NODE_H
#define PATH_PLANNING_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"  
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp" 
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"  
#include "visualization_msgs/msg/marker.hpp"        
#include "AStar.h"
#include "GridMap.h"
#include "traj_optmizer.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
#include <Eigen/Dense>

#include <unordered_set>
#include <cstdint>
#include <string>

class PathPlanningNode : public rclcpp::Node {
public:
    PathPlanningNode();
private:
    // 在节点启动时读取static_odom_to_map变换
    void initializeOdomToMapTransform();
    // 设置起点
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // 接收Point-LIO估计的线性加速度
    void accelerationCallback(
        const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);
    // 设置障碍物
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    // 设置目标
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    // 规划路径，主要
    void planPath(const MapNode& start, const MapNode& goal);
    // 发布路径，使用 Eigen::Vector2d 格式来包含二维位置 
    void publishPath(const std::vector<Eigen::Vector2d>& path);
    // 发布轨迹，使用 TrajOpt::Sample 格式来包含位置/速度/时间
    void publishTraj(const std::vector<TrajOpt::Sample>& path);
    // 发布局部栅格地图
    void mapPublishTimerCallback();
    // 发布安全走廊
    void publishCorridors(const std::vector<SafeTravelCorridor2D::Polyhedron2D>& corridors);
    // 发布栅格单元可视化
    void publishGridCells(const std::shared_ptr<GridMap>& map);
    // 设置仿真障碍物
    void obstacle_sim(const std::shared_ptr<GridMap>& map);
    // 设置仿真机器人
    void simulateRobotMotion();
    // 从PCD文件加载全局地图
    void loadGlobalMapFromPCD();
    // 滚动窗口更新走廊与轨迹
    void updateRollingOptimization();

    // TF2相关
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 订阅器
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acceleration_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
    // 订阅来自go_go_go节点的MPC轨迹速度信息
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mpc_velocity_subscriber_;
 
    // 路径的发布器
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    // 地图的发布器
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    // 静态PCD点云可视化发布器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_pcd_publisher_;
    // 走廊的发布器
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_publisher_;
    // 可视化栅格的发布器
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr grid_cells_publisher_;
    // 轨迹速度可视化发布器（用于RViz显示速度向量）
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajvel_visual_publisher_;
    // 轨迹速度发布器（每个轨迹点带速度信息，适合下发给控制器）
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_publisher_;

    // 定时器
    rclcpp::TimerBase::SharedPtr map_publish_timer_;
    rclcpp::TimerBase::SharedPtr rolling_opt_timer_;
    // A*算法相关
    std::shared_ptr<GridMap> grid_map_;
    std::unique_ptr<AStar> astar_;

    // 地图相关
    MapNode current_position_;                               // 起点
    Eigen::Vector2d current_position_map_ = Eigen::Vector2d::Zero(); // 实际位置，单位：m
    Eigen::Vector2d current_velocity_map_ = Eigen::Vector2d::Zero(); // 当前速度，单位：m/s
    Eigen::Vector2d current_acceleration_map_ = Eigen::Vector2d::Zero(); // Point-LIO估计加速度，单位：m/s^2
    bool has_motion_state_ = false;
    bool has_acceleration_state_ = false;

    MapNode current_goal_;
    bool has_goal_ = false;
    std::vector<TrajOpt::Sample> active_trajectory_;
    bool has_active_trajectory_ = false;
    rclcpp::Time last_replan_time_;
    bool replan_pending_ = false;

    float resolution = 10;                                   // 分辨率即放大倍数
    double odom_to_map_x, odom_to_map_y, odom_to_map_yaw;     // odom->map变换参数
    double maptransform_x_, maptransform_y_;     // globalmap->pointcloudmap的平移对齐微调
    int grid_radius_;                                     // GridMap膨胀/保存区域半径
    int map_size_x_ ;                                    // GridMap 宽（格子数）
    int map_size_y_ ;                                    // GridMap 高（格子数）

    // 安全走廊方法
    std::unique_ptr<SafeTravelCorridor2D> corridor_generator_;

    // MINCO
    std::unique_ptr<TrajOpt> trajopt;

    // 点云过滤高度
    double global_z_min;
    double global_z_max;
  
    double local_z_min;  
    double local_z_max;

  
    // 记录从PCD加载的静态障碍格子（键值 = (x<<32) | y）
    std::unordered_set<int64_t> static_obstacles_set_;

    bool enable_load_global_map_from_pcd;
};

#endif // PATH_PLANNING_NODE_H