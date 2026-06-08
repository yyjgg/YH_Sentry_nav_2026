#include "PathPlanningNode.h"
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <limits>
#include <cstdint>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.hpp>  // 添加用于创建四元数的头文件
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // 添加tf2转换头文件

PathPlanningNode::PathPlanningNode() : Node("path_planning_node") {
    this->declare_parameter<double>("odom_to_map_x");
    this->declare_parameter<double>("odom_to_map_y");
    this->declare_parameter<double>("odom_to_map_yaw");
    this->declare_parameter<double>("maptransform_x");
    this->declare_parameter<double>("maptransform_y");
    this->declare_parameter<double>("global_z_min");
    this->declare_parameter<double>("global_z_max");
    this->declare_parameter<double>("local_z_min");
    this->declare_parameter<double>("local_z_max");
    this->declare_parameter<int>("grid_radius");
    this->declare_parameter<int>("map_size_x");
    this->declare_parameter<int>("map_size_y");
    this->declare_parameter<bool>("enable_load_global_map_from_pcd");  // 默认为false

    this->get_parameter("odom_to_map_x", odom_to_map_x);
    this->get_parameter("odom_to_map_y", odom_to_map_y);
    this->get_parameter("odom_to_map_yaw", odom_to_map_yaw);
    this->get_parameter("maptransform_x", maptransform_x_);
    this->get_parameter("maptransform_y", maptransform_y_);
    this->get_parameter("global_z_min", global_z_min);
    this->get_parameter("global_z_max", global_z_max);
    this->get_parameter("local_z_min", local_z_min);
    this->get_parameter("local_z_max", local_z_max);
    this->get_parameter("grid_radius", grid_radius_);
    this->get_parameter("map_size_x", map_size_x_);
    this->get_parameter("map_size_y", map_size_y_);
    this->get_parameter("enable_load_global_map_from_pcd", enable_load_global_map_from_pcd);

    // 初始化TF2相关
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 在节点启动时读取static_odom_to_map变换
    initializeOdomToMapTransform();

    // 订阅Odometry话题
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 10,
        std::bind(&PathPlanningNode::odometryCallback, this, std::placeholders::_1));
    
    // 订阅PointCloud2话题
    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 10,
        std::bind(&PathPlanningNode::pointcloudCallback, this, std::placeholders::_1));
    
    // 订阅rviz2的2D Goal话题
    goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>( "/goal_pose", 10,
        std::bind(&PathPlanningNode::goalCallback, this, std::placeholders::_1));
    
    // 发布路径话题
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
 
    // 发布栅格地图话题
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/obstacle_grid_map", 10);

    // 创建安全走廊可视化发布器
    corridor_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/safe_corridors", 10);

    // 创建栅格单元可视化发布器（每个占用格显示为一个小立方体）
    grid_cells_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/grid_cells", 10);

    // 轨迹（带速度）发布器
    traj_publisher_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/planned_trajectory", 10);

    // 轨迹速度可视化发布器（用于RViz显示速度向量）
    trajvel_visual_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/trajectory_vel", 10);
    
    // 初始化地图（根据实际需求调整大小）
    grid_map_ = std::make_shared<GridMap>(map_size_x_, map_size_y_, grid_radius_);

    // 如果还没有接收到里程计，默认将起点设置为odom设定原点
    current_position_.x = odom_to_map_x * resolution;
    current_position_.y = odom_to_map_y * resolution;
    
    // 初始化A*算法
    astar_ = std::make_unique<AStar>(*grid_map_);
    
    // 初始化安全走廊生成器
    corridor_generator_ = std::make_unique<SafeTravelCorridor2D>(grid_map_);

    // 初始化轨迹优化器
    trajopt = std::make_unique<TrajOpt>();

    // 设置地图发布定时器（2Hz频率）
    map_publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PathPlanningNode::mapPublishTimerCallback, this));
    
    // 根据参数决定是否从PCD文件加载全局地图
    if (enable_load_global_map_from_pcd) {
        loadGlobalMapFromPCD();
        RCLCPP_INFO(this->get_logger(), "Loaded global map from PCD file");
    } else {
        RCLCPP_INFO(this->get_logger(), "Skipped loading global map from PCD file based on parameter setting");
    }

    RCLCPP_INFO(this->get_logger(), "PathPlanningNode initialized successfully");
}


// 在节点启动时读取static_odom_to_map变换
void PathPlanningNode::initializeOdomToMapTransform() {
    try {
        // 等待TF变换可用，最多等待5秒
        geometry_msgs::msg::TransformStamped odom_to_map_transform = 
            tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero, tf2::durationFromSec(5.0));
        
        // 提取平移和旋转信息（odom坐标系原点在map坐标系中的坐标）
        odom_to_map_x = odom_to_map_transform.transform.translation.x;
        odom_to_map_y = odom_to_map_transform.transform.translation.y;
        
        // 将四元数转换为欧拉角获取yaw
        tf2::Quaternion quat;
        tf2::fromMsg(odom_to_map_transform.transform.rotation, quat);
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        odom_to_map_yaw = yaw;
        
        RCLCPP_INFO(this->get_logger(), 
                   "Successfully initialized odom->map transform: x=%f, y=%f, yaw=%f", 
                   odom_to_map_x, odom_to_map_y, odom_to_map_yaw);
        
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), 
                    "Failed to lookup transform from odom to map: %s", ex.what());
        RCLCPP_WARN(this->get_logger(), 
                   "Using default values");
        
        // 使用参数文件中的默认值
        RCLCPP_WARN(this->get_logger(),
                "Keeping configured odom->map defaults: x=%f, y=%f, yaw=%f",
                odom_to_map_x, odom_to_map_y, odom_to_map_yaw);
    }
}
// 设置起点（机器人自身位置）
void PathPlanningNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 将odom坐标系下的位置转换到map坐标系
    double raw_x = msg->pose.pose.position.x + odom_to_map_x;
    double raw_y = msg->pose.pose.position.y + odom_to_map_y;
    
    // 转换为栅格坐标
    int grid_x = static_cast<int>(raw_x * resolution);
    int grid_y = static_cast<int>(raw_y * resolution);
    
    // 检查当前位置是否为障碍物（包括原始障碍物或膨胀障碍物）
    if (grid_map_->isWithinBounds(grid_x, grid_y) && 
        (grid_map_->isObstacle(grid_x, grid_y) || grid_map_->isInflatedObstacle(grid_x, grid_y))) {
        // 当前位置是障碍物，在5×5范围内寻找非障碍物点
        bool found_valid_point = false;
        int search_radius = 5; // 5×5范围，半径2
        
        // 从内到外搜索非障碍物点
        for (int radius = 1; radius <= search_radius; ++radius) {
            for (int dx = -radius; dx <= radius; ++dx) {
                for (int dy = -radius; dy <= radius; ++dy) {
                    // 跳过内层已经搜索过的点
                    if (std::abs(dx) < radius && std::abs(dy) < radius) {
                        continue;
                    }
                    int candidate_x = grid_x + dx;
                    int candidate_y = grid_y + dy;
                    if (grid_map_->isWithinBounds(candidate_x, candidate_y) && 
                        !grid_map_->isObstacle(candidate_x, candidate_y) &&
                        !grid_map_->isInflatedObstacle(candidate_x, candidate_y)) {
                        // 找到非障碍物点，使用该点作为起点
                        current_position_.x = candidate_x;
                        current_position_.y = candidate_y;
                        found_valid_point = true;
                        // RCLCPP_INFO(this->get_logger(), "real start point at (%d, %d)", candidate_x, candidate_y);
                        break;
                    }
                }
                if (found_valid_point) break;
            }
            if (found_valid_point) break;
        }
        if (!found_valid_point) {
            current_position_.x = grid_x;
            current_position_.y = grid_y;
        }
    } else {
        // 当前位置不是障碍物，直接使用
        current_position_.x = grid_x;
        current_position_.y = grid_y;
    }
}
// 设置局部地图障碍物
void PathPlanningNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 检查是否是第一次接收到点云消息
    static auto last_update_time = this->now();
    auto current_time = this->now();
    
    // 如果距离上次更新不足0.5秒，则跳过这次更新
    if ((current_time - last_update_time).seconds() < 0.5) {
        return;
    }
    
    // 更新最后更新时间
    last_update_time = current_time;
    // 处理点云数据，更新障碍物地图
    std::vector<int> current_obstacles;
    auto now = this->now();

    // 解析点云数据
    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
    
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        // 检查z坐标是否在0.1m到1m范围内
        if (*iter_z > local_z_min && *iter_z < local_z_max) {
            // 将odom坐标系下的点云坐标转换到map坐标系
            double map_x = *iter_x + odom_to_map_x;
            double map_y = *iter_y + odom_to_map_y;
            
            // 将坐标放大10倍并转换为栅格坐标
            int grid_x = static_cast<int>(map_x * resolution);
            int grid_y = static_cast<int>(map_y * resolution);
            
            // 检查坐标是否在地图范围内
            if (grid_map_->isWithinBounds(grid_x, grid_y)) {
                // 记录当前消息包设置的障碍物坐标
                current_obstacles.push_back(grid_x);
                current_obstacles.push_back(grid_y);
            }  
        }
    }
/***********************提前建全局地图需注释掉-start*******************************/
    // 创建一个临时的障碍物集合，用于快速查找
    std::unordered_set<int64_t> current_obstacles_set;
    for (size_t i = 0; i < current_obstacles.size(); i += 2) {
        int grid_x = current_obstacles[i];
        int grid_y = current_obstacles[i + 1];
        // 使用坐标对生成唯一键值
        int64_t key = (static_cast<int64_t>(grid_x) << 32) | static_cast<uint32_t>(grid_y);
        current_obstacles_set.insert(key);
    }

    // 动态障碍物处理 - 清除不再存在的障碍物
    for (auto& old_obstacles : grid_map_->history) {
        for (size_t i = 0; i < old_obstacles.size(); i += 2) {
            int old_x = old_obstacles[i];
            int old_y = old_obstacles[i + 1];

            // 生成键值
            int64_t key = (static_cast<int64_t>(old_x) << 32) | static_cast<uint32_t>(old_y);

            // 如果是PCD加载的静态障碍，则跳过，不要被局部刷新清除
            if (static_obstacles_set_.find(key) != static_obstacles_set_.end()) {
                continue;
            }

            // 如果当前帧中没有检测到该障碍物，则清除它（仅限动态障碍）
            if (current_obstacles_set.find(key) == current_obstacles_set.end()) {
                if (grid_map_->isWithinBounds(old_x, old_y)) {
                    // 检查是否是原始障碍物（静态障碍的膨胀区）
                    // 只清除动态障碍物，不触碰静态障碍物的膨胀区域
                    if (grid_map_->getGridCellValue(old_x, old_y) == 2) { // 膨胀区域
                        // 检查周围是否有原始障碍物（值为1）
                        bool hasStaticNearby = false;
                        for (int dx = -1; dx <= 1; dx++) {
                            for (int dy = -1; dy <= 1; dy++) {
                                int nx = old_x + dx;
                                int ny = old_y + dy;
                                if (grid_map_->isWithinBounds(nx, ny)) {
                                    // 检查是否是静态障碍物或者是静态障碍物的膨胀区域
                                    int64_t neighbor_key = (static_cast<int64_t>(nx) << 32) | static_cast<uint32_t>(ny);
                                    if (static_obstacles_set_.find(neighbor_key) != static_obstacles_set_.end() && 
                                        grid_map_->getGridCellValue(nx, ny) == 1) {
                                        hasStaticNearby = true;
                                        break;
                                    }
                                }
                            }
                            if (hasStaticNearby) break;
                        }
                        
                        // 如果附近有静态障碍物，不清理这个膨胀区域
                        if (!hasStaticNearby) {
                            grid_map_->setObstacle(old_x, old_y, false);
                        }
                    } else {
                        // 清除非膨胀区域的动态障碍
                        grid_map_->setObstacle(old_x, old_y, false);
                    }
                }
            }
        }
    }

    // 清空历史记录
    grid_map_->history.clear();
/****************提前建全局地图需注释掉-end****************************************/

    // 设置当前帧的新障碍物（并仅将动态障碍加入历史记录）
    std::vector<int> dynamic_obstacles;
    for (size_t i = 0; i < current_obstacles.size(); i += 2) {
        int grid_x = current_obstacles[i];
        int grid_y = current_obstacles[i + 1];
        if (!grid_map_->isWithinBounds(grid_x, grid_y)) continue;

        int64_t key = (static_cast<int64_t>(grid_x) << 32) | static_cast<uint32_t>(grid_y);

        // 标记为占用
        grid_map_->setObstacle(grid_x, grid_y, true);

        // 仅将非静态（非PCD）栅格加入动态历史
        if (static_obstacles_set_.find(key) == static_obstacles_set_.end()) {
            // 未在静态历史中找到，添加到动态障碍物
            dynamic_obstacles.push_back(grid_x);
            dynamic_obstacles.push_back(grid_y);
        }
    }

    // 将当前动态障碍物坐标加入历史记录
    if (!dynamic_obstacles.empty()) {
        grid_map_->history.push_back(dynamic_obstacles);
    }

    
}

// 设置目标
void PathPlanningNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 处理新的目标点（目标点已经在map坐标系下）
    MapNode goal(msg->pose.position.x * resolution, msg->pose.position.y * resolution);

    double goal_map_x = goal.x / resolution;
    double goal_map_y = goal.y / resolution;

    RCLCPP_INFO(this->get_logger(), "New goal received (grid): (%f, %f), (m): (%f, %f)", 
               goal.x, goal.y, goal_map_x, goal_map_y);

    RCLCPP_DEBUG(this->get_logger(), "Current start (grid): (%f, %f)", current_position_.x, current_position_.y);

    // 执行A*路径规划
    planPath(current_position_, goal);
}

// 规划路径，主要
void PathPlanningNode::planPath(const MapNode& start, const MapNode& goal) {
    // 诊断信息：打印起点/终点栅格坐标及对应是否为障碍或越界
    int sx = static_cast<int>(start.x);
    int sy = static_cast<int>(start.y);
    int gx = static_cast<int>(goal.x);
    int gy = static_cast<int>(goal.y);

    bool start_in_bounds = grid_map_->isWithinBounds(sx, sy);
    bool goal_in_bounds = grid_map_->isWithinBounds(gx, gy);
    bool start_is_obst = start_in_bounds ? grid_map_->isObstacle(sx, sy) : false;
    bool goal_is_obst = goal_in_bounds ? grid_map_->isObstacle(gx, gy) : false;
    bool start_is_inflated = start_in_bounds ? grid_map_->isInflatedObstacle(sx, sy) : false;
    bool goal_is_inflated = goal_in_bounds ? grid_map_->isInflatedObstacle(gx, gy) : false;

    RCLCPP_INFO(this->get_logger(), "Planning: start(grid)=(%d,%d) in_bounds=%d is_obstacle=%d is_inflated=%d; goal(grid)=(%d,%d) in_bounds=%d is_obstacle=%d is_inflated=%d",
               sx, sy, (int)start_in_bounds, (int)start_is_obst, (int)start_is_inflated,
               gx, gy, (int)goal_in_bounds, (int)goal_is_obst, (int)goal_is_inflated);

    if (!start_in_bounds || !goal_in_bounds) {
        RCLCPP_WARN(this->get_logger(), "Start or goal position out of map bounds");
        return;
    }

    if (start_is_obst || start_is_inflated) {
        RCLCPP_WARN(this->get_logger(), "Start position is inside an obstacle or inflated area (grid %d,%d)", sx, sy);
    }
    if (goal_is_obst || goal_is_inflated) {
        RCLCPP_WARN(this->get_logger(), "Goal position is inside an obstacle or inflated area (grid %d,%d)", gx, gy);
    }

    // 保存起点和终点周围区域的原始状态
    std::vector<std::tuple<int, int, int>> start_saved_states;
    std::vector<std::tuple<int, int, int>> goal_saved_states;
    
    RCLCPP_INFO(this->get_logger(), "Saving states around start and goal...");
    grid_map_->saveAreaState(sx, sy, grid_radius_, start_saved_states);  // 保存起点周围状态
    grid_map_->saveAreaState(gx, gy, grid_radius_, goal_saved_states);   // 保存终点周围状态
    
    // 检查保存了多少个状态
    RCLCPP_INFO(this->get_logger(), "Saved %zu states around start at (%d, %d)", start_saved_states.size(), sx, sy);
    RCLCPP_INFO(this->get_logger(), "Saved %zu states around goal at (%d, %d)", goal_saved_states.size(), gx, gy);
    
    // 临时清除起点和终点周围区域的膨胀效果，但保留原始障碍物
    RCLCPP_INFO(this->get_logger(), "Clearing inflated areas temporarily...");
    grid_map_->clearInflatedAreaOnly(sx, sy, grid_radius_);  // 清除起点周围膨胀区域
    grid_map_->clearInflatedAreaOnly(gx, gy, grid_radius_);  // 清除终点周围膨胀区域
    
    // 执行A*搜索得到原始路径
    auto original_path = astar_->Search(start, goal);
    
    if (!original_path.empty()) {
        // 提取关键点简化路径
        auto simplified_path = astar_->extractKeyPoints(original_path, 5.0); 
        std::vector<Eigen::Vector2d> path;
        for(auto &p:simplified_path){
            path.emplace_back(p.x,p.y);
        }

        // 生成安全走廊
        auto corridors = corridor_generator_->buildCorridor(path,32,10);
        
        // 发布安全走廊可视化
        publishCorridors(corridors);

        auto traj = trajopt->Planning(path,corridors);

        // 发布简化后的路径/轨迹可视化（traj 现在包含位置+速度+时间）
        publishTraj(traj);
        // publishPath(path);

    } else {
        RCLCPP_WARN(this->get_logger(), "No path found from (%f, %f) to (%f, %f)", 
                    start.x, start.y, goal.x, goal.y);
    
        // 清空路径
        auto empty_path = std::make_unique<nav_msgs::msg::Path>();
        empty_path->header.stamp = this->now();
        empty_path->header.frame_id = "map";
        path_publisher_->publish(std::move(empty_path));
    
        // 清空轨迹
        auto empty_traj = std::make_unique<trajectory_msgs::msg::MultiDOFJointTrajectory>();
        empty_traj->header.stamp = this->now();
        empty_traj->header.frame_id = "map";
        traj_publisher_->publish(std::move(empty_traj));
    
        // 清空速度可视化
        visualization_msgs::msg::Marker clear_vel;
        clear_vel.header.stamp = this->now();
        clear_vel.header.frame_id = "map";
        clear_vel.action = visualization_msgs::msg::Marker::DELETEALL;
        trajvel_visual_publisher_->publish(clear_vel);
        
    }
    
    // 恢复起点和终点周围区域的原始状态
    RCLCPP_INFO(this->get_logger(), "Restoring original inflated areas...");
    grid_map_->restoreAreaState(start_saved_states);  // 恢复起点周围状态
    grid_map_->restoreAreaState(goal_saved_states);   // 恢复终点周围状态
    
    RCLCPP_INFO(this->get_logger(), "Restored original inflated areas around start and goal");
    
    // 验证恢复是否成功
    bool start_restored = grid_map_->isInflatedObstacle(sx, sy) == start_is_inflated && 
                          grid_map_->isObstacle(sx, sy) == start_is_obst;
    bool goal_restored = grid_map_->isInflatedObstacle(gx, gy) == goal_is_inflated && 
                         grid_map_->isObstacle(gx, gy) == goal_is_obst;
    
    RCLCPP_INFO(this->get_logger(), "Verification: Start restored correctly: %s, Goal restored correctly: %s", 
                start_restored ? "true" : "false", 
                goal_restored ? "true" : "false");
}



// 发布路径
void PathPlanningNode::publishPath(const std::vector<Eigen::Vector2d>& path) {
    auto path_msg = std::make_unique<nav_msgs::msg::Path>();
    path_msg->header.stamp = this->now();
    path_msg->header.frame_id = "map";

    // 填充路径位姿（栅格->米）
    for (const auto& point : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg->header;
        pose.pose.position.x = point.x() / resolution;
        pose.pose.position.y = point.y() / resolution;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path_msg->poses.push_back(pose);
    }

    // 发布仅包含位姿的路径
    path_publisher_->publish(std::move(path_msg));
}
void PathPlanningNode::publishTraj(const std::vector<TrajOpt::Sample>& traj) {
    auto path_msg = std::make_unique<nav_msgs::msg::Path>();
    path_msg->header.stamp = this->now();
    path_msg->header.frame_id = "map";

    // 填充路径位姿（栅格->米）
    for (const auto& sample : traj) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg->header;
        pose.pose.position.x = sample.pos.x() / resolution;
        pose.pose.position.y = sample.pos.y() / resolution;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path_msg->poses.push_back(pose);
    }

    // 发布仅包含位姿的路径
    path_publisher_->publish(std::move(path_msg));

    // 生成并发布带速度的轨迹（MultiDOFJointTrajectory）
    if (traj.empty()) {
        RCLCPP_WARN(this->get_logger(), "Attempted to publish empty trajectory!");
        return;
    }

    trajectory_msgs::msg::MultiDOFJointTrajectory traj_msg;
    traj_msg.header.stamp = this->now();
    traj_msg.header.frame_id = "map";
    traj_msg.joint_names.push_back("base_link");

    // 创建一个副本，对终点速度进行特殊处理
    std::vector<TrajOpt::Sample> processed_traj = traj;
    
    // 在轨迹末尾添加几个速度逐渐减小的点，确保机器人平稳停下
    if (processed_traj.size() > 2) {
        // 获取倒数第二个样本作为基准
        auto& last_sample = processed_traj.back();
        auto penultimate_sample = *(processed_traj.rbegin()+1);
        
        // 在最后添加2个额外的点，速度逐渐减小到0
        TrajOpt::Sample final_stop_point = last_sample;
        final_stop_point.vel = Eigen::Vector2d(0.0, 0.0);  // 最终速度为0
        final_stop_point.t = last_sample.t + 0.1;  // 时间稍微延后
        processed_traj.push_back(final_stop_point);
        
        // 再添加一个完全停止的点
        TrajOpt::Sample extra_stop_point = final_stop_point;
        extra_stop_point.t = final_stop_point.t + 0.2;  // 再延后0.2秒
        processed_traj.push_back(extra_stop_point);
    }

    for (const auto &sample : processed_traj) {
        double x = sample.pos.x() / resolution; // 转为米
        double y = sample.pos.y() / resolution;
        double vx = sample.vel.x() / resolution; // 转为 m/s
        double vy = sample.vel.y() / resolution;

        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;

        geometry_msgs::msg::Transform tform;
        tform.translation.x = x;
        tform.translation.y = y;
        tform.translation.z = 0.0;
        tform.rotation.w = 1.0;
        point.transforms.push_back(tform);

        geometry_msgs::msg::Twist twist;
        twist.linear.x = vx;
        twist.linear.y = vy;
        twist.linear.z = 0.0;
        point.velocities.push_back(twist);

        // time_from_start 使用 sample.t
        double t = sample.t;
        builtin_interfaces::msg::Duration d;
        d.sec = static_cast<int32_t>(std::floor(t));
        d.nanosec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
        point.time_from_start = d;

        traj_msg.points.push_back(point);
    }

    traj_publisher_->publish(std::move(traj_msg));

    // 打印前若干点到终端以便快速验证
    // size_t print_n = std::min<size_t>(50, processed_traj.size());
    size_t print_n = processed_traj.size();
    RCLCPP_INFO(this->get_logger(), "Published trajectory with %zu points (showing first %zu):", processed_traj.size(), print_n);
    for (size_t i = 0; i < print_n; ++i) {
        const auto &s = processed_traj[i];
        double px = s.pos.x() / resolution;
        double py = s.pos.y() / resolution;
        double vx = s.vel.x() / resolution;
        double vy = s.vel.y() / resolution;
        double t = s.t;
        // RCLCPP_INFO(this->get_logger(), "  pt %zu: pos=(%.3f,%.3f) vel=(%.3f,%.3f) t=%.3f", i, px, py, vx, vy, t);
    }

    // 在 RViz 中显示速度向量（使用 LINE_LIST，每对点表示起点->速度终点）
    visualization_msgs::msg::Marker vel_marker;
    vel_marker.header.stamp = this->now();
    vel_marker.header.frame_id = "map";
    vel_marker.ns = "traj_vel";
    vel_marker.id = 0;
    vel_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    vel_marker.action = visualization_msgs::msg::Marker::ADD;
    vel_marker.scale.x = 0.02; // 线宽
    vel_marker.color.r = 1.0;
    vel_marker.color.g = 0.0;
    vel_marker.color.b = 0.0;
    vel_marker.color.a = 0.9;

    // 缩放因子用于将速度可视化成合适长度（秒）
    double vis_scale_s = 0.5;
    for (const auto &s : processed_traj) {
        geometry_msgs::msg::Point start, end;
        start.x = s.pos.x() / resolution;
        start.y = s.pos.y() / resolution;
        start.z = 0.0;
        end.x = start.x + (s.vel.x() / resolution) * vis_scale_s;
        end.y = start.y + (s.vel.y() / resolution) * vis_scale_s;
        end.z = 0.0;
        vel_marker.points.push_back(start);
        vel_marker.points.push_back(end);
    }

    if (!vel_marker.points.empty()) {
        trajvel_visual_publisher_->publish(std::move(vel_marker));
    }
    
    // 不再在这里发布动态点，改用定时器发布
}

void PathPlanningNode::loadGlobalMapFromPCD(){
    // 加载PCD文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 优先从已安装/共享目录加载包资源；如果找不到则回退到相对路径
    std::string pcd_file_path;
    auto share_dir = ament_index_cpp::get_package_share_directory("small_point_lio");
    pcd_file_path = share_dir + "/pcd/RMUC2026.pcd";
     
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", pcd_file_path.c_str());
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Loaded %lu points from PCD file", cloud->size());

    // 初始化xyz坐标范围
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();
    // 遍历点云数据，将点投影到2D栅格地图上
    for (const auto& point : cloud->points) {
        // 更新x，y，z的最大值和最小值
        min_x = std::min(min_x, static_cast<double>(point.x));
        max_x = std::max(max_x, static_cast<double>(point.x));
        min_y = std::min(min_y, static_cast<double>(point.y));
        max_y = std::max(max_y, static_cast<double>(point.y));
        min_z = std::min(min_z, static_cast<double>(point.z));
        max_z = std::max(max_z, static_cast<double>(point.z));
        // 只处理z轴在0.1m到1m范围内的点
        if (- point.z > global_z_min && - point.z < global_z_max) {
            // 将点云坐标转换为栅格坐标
            int grid_x = static_cast<int>((point.x + maptransform_x_)  * resolution );
            int grid_y = static_cast<int>((point.y + maptransform_y_)  * resolution );
            
            // 检查坐标是否在地图范围内
            if (grid_map_->isWithinBounds(grid_x, grid_y)) {
                // 设置为障碍物
                grid_map_->setObstacle(grid_x, grid_y, true);
                // 记录为静态障碍，后续局部刷新不应清除
                int64_t key = (static_cast<int64_t>(grid_x) << 32) | static_cast<uint32_t>(grid_y);
                static_obstacles_set_.insert(key);
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "Point cloud XYZ range: X(%.2f ~ %.2f), Y(%.2f ~ %.2f), Z(%.2f ~ %.2f)", 
                min_x, max_x, min_y, max_y, min_z, max_z);

    RCLCPP_INFO(this->get_logger(), "Global map loaded from PCD file and converted to occupancy grid");

}

// 发布栅格地图
void PathPlanningNode::mapPublishTimerCallback() {
    auto grid_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();    
    grid_msg->header.stamp = this->now();
    grid_msg->header.frame_id = "map";
    grid_msg->info.resolution = 1.0 / resolution;  // 栅格大小（米）
    grid_msg->info.width = grid_map_->getSizeX();
    grid_msg->info.height = grid_map_->getSizeY();
    
    // 设置地图原点（根据实际需求调整）
    grid_msg->info.origin.position.x = 0.0;
    grid_msg->info.origin.position.y = 0.0;
    grid_msg->info.origin.position.z = 0.0;
    grid_msg->info.origin.orientation.w = 1.0;
    
    grid_msg->data.resize(grid_msg->info.width * grid_msg->info.height, 0);

    // 自行设置障碍物用于测试
    // obstacle_sim(grid_map_);
    
    // 填充地图数据
    for (int x = 0; x < grid_map_->getSizeX(); ++x) {
        for (int y = 0; y < grid_map_->getSizeY(); ++y) {
            int index = y * grid_msg->info.width + x;
            if (grid_map_->isObstacle(x, y) == true) {
                grid_msg->data[index] = 100;
            }
            else if (grid_map_->isInflatedObstacle(x, y) == true){
                grid_msg->data[index] = 50;
            }
            else
            grid_msg->data[index] = 0;

        }
    }

    // 发布栅格可视化
    publishGridCells(grid_map_);

    map_publisher_->publish(std::move(grid_msg));
    RCLCPP_DEBUG(this->get_logger(), "Grid map published with size %dx%d", 
                grid_msg->info.width, grid_msg->info.height);

}

// 栅格可视化
void PathPlanningNode::publishGridCells(const std::shared_ptr<GridMap>& map)
{
    auto marker = std::make_unique<visualization_msgs::msg::Marker>();
    
    marker->header.stamp = this->now();
    marker->header.frame_id = "map";
    marker->ns = "gridcells";
    marker->id = 0;
    marker->type = visualization_msgs::msg::Marker::LINE_LIST;
    marker->action = visualization_msgs::msg::Marker::ADD;
    
    // 设置线宽
    marker->scale.x = 0.01;  // 线宽
    
    // 设置颜色（蓝色）
    marker->color.r = 0.2;
    marker->color.g = 0.2;
    marker->color.b = 1.0;
    marker->color.a = 0.7;

    // 计算地图物理尺寸
    double width_m = map->getSizeX() * (1.0 / resolution);
    double height_m = map->getSizeY() * (1.0 / resolution);

    // 抬高到0.5米高度（避免地面遮挡）
    double z_height = 0.5;
    
    // 添加水平线
    for (int y = 0; y <= map->getSizeY(); ++y) {
        double y_coord = y * (1.0 / resolution);
        
        // 左端点
        geometry_msgs::msg::Point p1;
        p1.x = 0.0;
        p1.y = y_coord;
        p1.z = 0.1;  // 稍微抬高，避免与地面重合
        
        // 右端点
        geometry_msgs::msg::Point p2;
        p2.x = map->getSizeX() * (1.0 / resolution);
        p2.y = y_coord;
        p2.z = 0.1;
        
        marker->points.push_back(p1);
        marker->points.push_back(p2);
    }
    
    // 添加垂直线
    for (int x = 0; x <= map->getSizeX(); ++x) {
        double x_coord = x * (1.0 / resolution);
        
        // 下端点
        geometry_msgs::msg::Point p1;
        p1.x = x_coord;
        p1.y = 0.0;
        p1.z = 0.1;
        
        // 上端点
        geometry_msgs::msg::Point p2;
        p2.x = x_coord;
        p2.y = map->getSizeY() * (1.0 / resolution);
        p2.z = 0.1;
        
        marker->points.push_back(p1);
        marker->points.push_back(p2);
    }
    
    grid_cells_publisher_->publish(std::move(marker));
    RCLCPP_DEBUG(this->get_logger(), "Grid cells published with size %dx%d", 
                map->getSizeX(), map->getSizeY());
}
// 发布安全走廊
void PathPlanningNode::publishCorridors(const std::vector<SafeTravelCorridor2D::Polyhedron2D>& corridors) {
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    
    // 清除之前的标记
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.header.stamp = this->now();
    clear_marker.ns = "corridors";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array->markers.push_back(clear_marker);
    
    // 创建单个marker包含所有走廊
    visualization_msgs::msg::Marker corridor_marker;
    corridor_marker.header.frame_id = "map";
    corridor_marker.header.stamp = this->now();
    corridor_marker.ns = "corridors";
    corridor_marker.id = 0;
    corridor_marker.type = visualization_msgs::msg::Marker::LINE_LIST;  // 线段列表
    corridor_marker.action = visualization_msgs::msg::Marker::ADD;
    
    // 样式设置
    corridor_marker.scale.x = 0.01;  // 线宽
    corridor_marker.color.r = 0.5;  // 深绿色
    corridor_marker.color.g = 0.0;
    corridor_marker.color.b = 0.5;
    corridor_marker.color.a = 1.0;
    
    // 为每个走廊添加线段
    for (const auto& corridor : corridors) {
        if (corridor.vertices.size() < 2) continue;
        
        // 添加多边形的边
        for (size_t i = 0; i < corridor.vertices.size(); ++i) {
            size_t next_i = (i + 1) % corridor.vertices.size();
            
            // 第一个点
            geometry_msgs::msg::Point p1;
            p1.x = corridor.vertices[i].x()/resolution;
            p1.y = corridor.vertices[i].y()/resolution;
            p1.z = 0.1;
            corridor_marker.points.push_back(p1);
            
            // 第二个点
            geometry_msgs::msg::Point p2;
            p2.x = corridor.vertices[next_i].x()/resolution;
            p2.y = corridor.vertices[next_i].y()/resolution;
            p2.z = 0.1;
            corridor_marker.points.push_back(p2);
        }
    }
    
    if (!corridor_marker.points.empty()) {
        marker_array->markers.push_back(corridor_marker);
    }
    
    corridor_publisher_->publish(std::move(marker_array));
    RCLCPP_INFO(this->get_logger(), "Published corridors with %zu points", 
                corridor_marker.points.size());
}

// 设置用于仿真的障碍物
void PathPlanningNode::obstacle_sim(const std::shared_ptr<GridMap>& map){
    // 在地图中添加一些模拟障碍物
    // 1. 创建一个L形墙壁障碍物
    for (int x = 30; x < 35; ++x) {
        for (int y = 30; y < 150; ++y) {
            map->setObstacle(x, y, true);
        }
    }
    
    for (int x = 35; x < 60; ++x) {
        for (int y = 30; y < 35; ++y) {
            map->setObstacle(x, y, true);
        }
    }
    
    // 2. 创建两个矩形障碍物
    for (int x = 65; x < 100; ++x) {
        for (int y = 50; y < 80; ++y) {
            map->setObstacle(x, y, true);
        }
    }
    
    for (int x = 65; x < 100; ++x) {
        for (int y = 100; y < 140; ++y) {
            map->setObstacle(x, y, true);
        }
    }
    
    // 3. 创建一个圆形障碍物（近似）
    int center_x = 120;
    int center_y = 60;
    int radius = 15;
    for (int x = center_x - radius; x <= center_x + radius; ++x) {
        for (int y = center_y - radius; y <= center_y + radius; ++y) {
            if ((x - center_x) * (x - center_x) + (y - center_y) * (y - center_y) <= radius * radius) {
                map->setObstacle(x, y, true);
            }
        }
    }
    
    // 4. 创建一个长条形障碍物（横向）
    for (int x = 140; x < 190; ++x) {
        for (int y = 100; y < 105; ++y) {
            map->setObstacle(x, y, true);
        }
    }
    
    // 5. 创建一些小的独立障碍物
    map->setObstacle(180, 30, true);
    map->setObstacle(200, 70, true);
    map->setObstacle(150, 150, true);

    // RCLCPP_INFO(this->get_logger(), "Simulated obstacles added to map");
}
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlanningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}