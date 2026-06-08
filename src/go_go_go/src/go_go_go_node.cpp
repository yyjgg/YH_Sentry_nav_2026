#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <iostream>
#include "go_go_go/SerialPort.h"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "visualization_msgs/msg/marker.hpp"  
#include <cmath>
#include <osqp/osqp.h>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>  // 添加Int16消息类型头文件


class GoGoGoNode : public rclcpp::Node
{
public:
    GoGoGoNode() : Node("go_go_go_node")
    {
        this->declare_parameter<double>("odom_to_map_x");
        this->declare_parameter<double>("odom_to_map_y");
        this->declare_parameter<double>("odom_to_map_yaw");
        this->declare_parameter<double>("sign_x");
        this->declare_parameter<double>("sign_y");
        this->declare_parameter<double>("left_patrol_x");
        this->declare_parameter<double>("left_patrol_y");
        this->declare_parameter<double>("right_patrol_x");
        this->declare_parameter<double>("right_patrol_y");
        this->declare_parameter<double>("back_patrol_x");
        this->declare_parameter<double>("back_patrol_y");
        this->declare_parameter<double>("front_patrol_x");
        this->declare_parameter<double>("front_patrol_y");
        this->declare_parameter<double>("origin_x");
        this->declare_parameter<double>("origin_y");
        this->declare_parameter<double>("max_lin_speed");
        this->declare_parameter<double>("max_lin_accel");

        this->get_parameter("odom_to_map_x", odom_to_map_x);
        this->get_parameter("odom_to_map_y", odom_to_map_y);
        this->get_parameter("odom_to_map_yaw", odom_to_map_yaw);
        this->get_parameter("sign_x", sign_x_);
        this->get_parameter("sign_y", sign_y_);
        this->get_parameter("left_patrol_x", left_patrol_x_);
        this->get_parameter("left_patrol_y", left_patrol_y_);
        this->get_parameter("right_patrol_x", right_patrol_x_);
        this->get_parameter("right_patrol_y", right_patrol_y_);
        this->get_parameter("back_patrol_x", back_patrol_x_);
        this->get_parameter("back_patrol_y", back_patrol_y_);
        this->get_parameter("front_patrol_x", front_patrol_x_);
        this->get_parameter("front_patrol_y", front_patrol_y_);
        this->get_parameter("origin_x", origin_x_);
        this->get_parameter("origin_y", origin_y_);
        this->get_parameter("max_lin_speed", max_lin_speed_);
        this->get_parameter("max_lin_accel", max_lin_accel_);

        // 初始化MPC参数
        mpc_horizon_ = 20;  // 减少预测时域，提高响应速度
        dt_ = 0.1;         // 减少时间步长，提高精度
        q_pos_ = 12.0;       // 位置跟踪权重，调整以平衡跟踪和平滑性
        q_yaw_ = 0.8;       // 增加航向角跟踪权重，更好地跟踪曲线方向
        r_ctrl_ = 0.3;      // 控制量速度输入权重
        r_dctrl_ = 0.15;    // 控制量速度变化权重，略微降低以允许更快响应
        r_tau_ = 0.1;       // 角速度控制权重
        
        // 物理参数初始化
        // mass_ = 20.0;       // 机器人质量
        // inertia_ = 4.0;     // 假设机器人绕z轴转动惯量 Iz = 1/12 * m *（ L^2 + W^2 ），L长 W宽

        // 初始化TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建订阅者
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&GoGoGoNode::odom_callback, this, std::placeholders::_1));
            
        path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10,
            std::bind(&GoGoGoNode::path_callback, this, std::placeholders::_1));

        traj_subscription_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
            "/planned_trajectory", 10,
            std::bind(&GoGoGoNode::traj_callback, this, std::placeholders::_1));
            
        // 命令订阅
        cmd_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
            "/target_number", 10,
            std::bind(&GoGoGoNode::cmd_callback, this, std::placeholders::_1));
            
        // 速度指令发布
        mpc_velocity_publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/target_pos", 10);
        
        // 创建目标点发布器（发布到 /goal_pose 话题）
        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        // 在节点启动时初始化变换
        initializeOdomToMapTransform();

        // 初始化速度限制与状态
        prev_cmd_ = Eigen::Vector2d::Zero();
        goal_reached_ = false;
        
        RCLCPP_INFO(this->get_logger(), "GoGoGo is running, it is listening to Odometry and planned_path topics.");
    }

private:
    // ===== 目标点发布函数 =====
    void publishGoal(double x, double y) {
        auto goal_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
        goal_msg->header.stamp = this->now();
        goal_msg->header.frame_id = "map";
        goal_msg->pose.position.x = x;
        goal_msg->pose.position.y = y;
        goal_msg->pose.position.z = 0.0;
        goal_msg->pose.orientation.w = 1.0;  // 默认朝向
        goal_publisher_->publish(std::move(goal_msg));
        RCLCPP_INFO(this->get_logger(), "Published goal: (%.2f, %.2f)", x, y);
    }
    // ===== 命令回调函数 =====
    void cmd_callback(const std_msgs::msg::Int16::SharedPtr msg) {
        switch(msg->data) {
            case 0:
                RCLCPP_INFO(this->get_logger(), "Received command 0: Stop moving");
                goal_reached_ = true;
                prev_cmd_ = Eigen::Vector2d::Zero();
                // 清空轨迹，表示当前任务已完成
                traj_matrix_ = Eigen::MatrixXd(0, 4);
                last_traj_index_ = 0;
                last_cmd_ = 0;  // 更新最后命令
                break;
                
            case 1:
                RCLCPP_INFO(this->get_logger(), "Received command 1: Return to origin point");
                if (!goal_reached_ || last_cmd_ != 1) {
                    publishGoal(origin_x_, origin_y_);
                }
                if (traj_matrix_.rows() != 0) {
                    last_cmd_ = 1;  // 更新最后命令
                }
                break;
      
            case 2:
                RCLCPP_INFO(this->get_logger(), "Received command 2: Go to sign point");
                if (!goal_reached_ || last_cmd_ != 2) {
                    publishGoal(sign_x_, sign_y_);
                }
                if (traj_matrix_.rows() != 0) {
                    last_cmd_ = 2;  // 更新最后命令
                }
                break;
            case 3:
                RCLCPP_INFO(this->get_logger(), "Received command 3: Go to left patrol point");
                if (!goal_reached_ || last_cmd_ != 3) {
                    publishGoal(left_patrol_x_, left_patrol_y_);
                }
                if (traj_matrix_.rows() != 0) {
                    last_cmd_ = 3;  // 更新最后命令
                }
                break;
            case 4:
                RCLCPP_INFO(this->get_logger(), "Received command 4: Go to right patrol point");
                // RCLCPP_INFO(this->get_logger(), "Current goal_reached status: %s", goal_reached_ ? "true" : "false");
                if (!goal_reached_ || last_cmd_ != 4) {
                    publishGoal(right_patrol_x_, right_patrol_y_);
                }
                if (traj_matrix_.rows() != 0) {
                    last_cmd_ = 4;  // 更新最后命令
                }
                break;
            case 5:
                RCLCPP_INFO(this->get_logger(), "Received command 5: Go to back patrol point ");
                if (!goal_reached_ || last_cmd_ != 5) {
                    publishGoal(back_patrol_x_, back_patrol_y_);
                }
                if (traj_matrix_.rows() != 0) {
                    last_cmd_ = 5;  // 更新最后命令
                }
                break;   
            case 6:
                RCLCPP_INFO(this->get_logger(), "Received command 6: Go to front patrol point");
                if (!goal_reached_ || last_cmd_ != 6) {
                    publishGoal(front_patrol_x_, front_patrol_y_);
                    // goal_reached_ = false;  // 重置到达状态
                    // last_traj_index_ = 0;   // 重置轨迹索引
                }
                if (traj_matrix_.rows() != 0) {
                    last_cmd_ = 6;  // 更新最后命令
                }
                break;
            
            case 7:
                // if (last_cmd_ != 7) {
                //     RCLCPP_INFO(this->get_logger(), "Received command 7: test");
                //     publishGoal(odom_to_map_x, odom_to_map_y);
                // }
                // last_cmd_ = 7;  // 更新最后命令
                break;

            default:
                last_cmd_ = msg->data;
                break;

        }
    }
    
    // ===== 坐标系转换工具函数 =====
    // 地图坐标系速度 -> 车体坐标系速度
    // 坐标系定义:
    //   地图系: X→北, Y→西
    //   车体系: X→前, Y→右
    // 转换矩阵: [v_body_x]   [cos(θ)  sin(θ)] [v_map_x]
    //          [v_body_y] = [sin(θ)  -cos(θ)] [v_map_y]
    Eigen::Vector2d mapFrameToBodyFrame(const Eigen::Vector2d& vel_map, double yaw) {
        double cos_yaw = cos(yaw);
        double sin_yaw = sin(yaw);
        
        // 标准旋转矩阵（用于自定义坐标系）
        Eigen::Vector2d vel_body;
        vel_body(0) = cos_yaw * vel_map(0) + sin_yaw * vel_map(1);
        vel_body(1) = sin_yaw * vel_map(0) - cos_yaw * vel_map(1);
        
        return vel_body;
    }
    
    // 发送速度给底层控制（自动进行坐标系转换）
    void sendVelocityCommand(const Eigen::Vector2d& vel_map, double yaw) {
        Eigen::Vector2d vel_body = mapFrameToBodyFrame(vel_map, yaw);
        auto velocity_msg = std_msgs::msg::Int16MultiArray();
        // 添加数据到数组
        velocity_msg.data.push_back(static_cast<int16_t>(1000 * vel_body(0)));
        velocity_msg.data.push_back(static_cast<int16_t>(1000 * vel_body(1)));
        mpc_velocity_publisher_->publish(velocity_msg);
        // dg.Vision2Control(1000 * vel_body(0), 1000 * vel_body(1));
        // serial_.send_data(dg);
        // RCLCPP_INFO(this->get_logger(), "Sent velocity command: [%f, %f]", vel_body(0), vel_body(1));
    }

    void initializeOdomToMapTransform() {
        RCLCPP_DEBUG(this->get_logger(), "Attempting to initialize odom->map transform");
        
        try {
            // 等待TF变换可用
            if (!tf_buffer_->canTransform("map", "odom", tf2::TimePointZero, 
                                         tf2::durationFromSec(5.0))) {
                RCLCPP_WARN(this->get_logger(), "Cannot transform from 'map' to 'odom'");
            }
            
            geometry_msgs::msg::TransformStamped odom_to_map_transform = 
                tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero, 
                                          tf2::durationFromSec(5.0));
            
            // 输出完整的变换信息用于调试
            RCLCPP_DEBUG(this->get_logger(), 
                        "Raw transform - Translation: (%f, %f, %f), "
                        "Rotation: (%f, %f, %f, %f)",
                        odom_to_map_transform.transform.translation.x,
                        odom_to_map_transform.transform.translation.y,
                        odom_to_map_transform.transform.translation.z,
                        odom_to_map_transform.transform.rotation.x,
                        odom_to_map_transform.transform.rotation.y,
                        odom_to_map_transform.transform.rotation.z,
                        odom_to_map_transform.transform.rotation.w);
            
            odom_to_map_x = odom_to_map_transform.transform.translation.x;
            odom_to_map_y = odom_to_map_transform.transform.translation.y;
            
            tf2::Quaternion quat;
            tf2::fromMsg(odom_to_map_transform.transform.rotation, quat);
            tf2::Matrix3x3 mat(quat);
            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);
            odom_to_map_yaw = yaw;
            
            RCLCPP_INFO(this->get_logger(), 
                       "Successfully initialized odom->map transform: x=%f, y=%f, yaw=%f (rad)", 
                       odom_to_map_x, odom_to_map_y, odom_to_map_yaw);
            RCLCPP_INFO(this->get_logger(), 
                       "Transform angles - Roll: %f, Pitch: %f, Yaw: %f (deg)", 
                       roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
            
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), 
                        "Failed to lookup transform from odom to map: %s", ex.what());
            RCLCPP_WARN(this->get_logger(), 
                       "Using default values");
            
            RCLCPP_WARN(this->get_logger(),
                        "Keeping configured odom->map defaults: x=%f, y=%f, yaw=%f",
                        odom_to_map_x, odom_to_map_y, odom_to_map_yaw);
        }
        
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        process_odometry(msg);
    }
    
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        process_path(msg);
    }

    void traj_callback(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg)
    { 
        RCLCPP_INFO(this->get_logger(), "Received trajectory message with %zu points", msg->points.size());
        process_traj(msg);
    }
    
    void process_odometry(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        // 获取当前位置和姿态
        double x = odom_msg->pose.pose.position.x + odom_to_map_x;
        double y = odom_msg->pose.pose.position.y + odom_to_map_y;
        
        // 获取姿态角（从四元数转为欧拉角）
        tf2::Quaternion q;
        tf2::fromMsg(odom_msg->pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        // 获取线速度和角速度
        double vx = odom_msg->twist.twist.linear.x;
        double vy = odom_msg->twist.twist.linear.y;
        double wyaw = odom_msg->twist.twist.angular.z;
          
        Eigen::Vector3d current_pos(x, y, yaw);
        Eigen::Vector3d current_vel(vx, vy, wyaw);
        
        // 如果之前已经判定到达目标，持续发布零速度并不再执行MPC
        if (goal_reached_) {
            // auto velocity_msg = geometry_msgs::msg::Twist();
            // velocity_msg.linear.x = 0.0;
            // velocity_msg.linear.y = 0.0;
            // velocity_msg.linear.z = 0.0;
            // velocity_msg.angular.x = 0.0;
            // velocity_msg.angular.y = 0.0;
            // velocity_msg.angular.z = 0.0;
            // mpc_velocity_publisher_->publish(velocity_msg);
            
            // 发送零速度给底层控制（车体坐标系）
            sendVelocityCommand(Eigen::Vector2d::Zero(), yaw);
            return;
        }

        // 检查路径矩阵是否为空
        if (traj_matrix_.rows() == 0) {
            // 即使没有轨迹，也发布零速度命令以确保cmd_vel有输出
            // auto velocity_msg = geometry_msgs::msg::Twist();
            // velocity_msg.linear.x = 0.0;
            // velocity_msg.linear.y = 0.0;
            // velocity_msg.linear.z = 0.0;
            // velocity_msg.angular.x = 0.0;
            // velocity_msg.angular.y = 0.0;
            // velocity_msg.angular.z = 0.0;
            
            // 清除上一命令，避免之后出现突变
            prev_cmd_ = Eigen::Vector2d::Zero();

            // mpc_velocity_publisher_->publish(velocity_msg);
            
            // 发送零速度给底层控制（车体坐标系）
            sendVelocityCommand(Eigen::Vector2d::Zero(), yaw);
            
            // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            //                      "No trajectory available, publishing zero velocity");
            return;
        }
        
        // 轨迹点查找：直接使用最近点作为MPC参考起点
        // MPC自带预测时域(N=20步)，Q矩阵的位置权重会驱动跟踪前方轨迹，
        // 不需要额外的pure pursuit前视——那会跳过近处轨迹点，造成信息丢失
        int start_index = std::max(0, last_traj_index_ - 10);
        int end_index = std::min(static_cast<int>(traj_matrix_.rows() - 1), last_traj_index_ + 50);

        double min_distance = 1e10;
        int target_index = start_index;

        for (int i = start_index; i <= end_index; i++) {
            Eigen::Vector2d traj_point(traj_matrix_(i, 0), traj_matrix_(i, 1));
            Eigen::Vector2d current_pos_2d(x, y);
            double distance = (current_pos_2d - traj_point).norm();

            if (distance < min_distance) {
                min_distance = distance;
                target_index = i;
            }
        }

        last_traj_index_ = target_index;
        
        if (target_index < 0 || target_index >= static_cast<int>(traj_matrix_.rows())) {
            RCLCPP_WARN(this->get_logger(), "Invalid trajectory index: %d", target_index);
            return;
        }
        
        // 获取终点位置和速度
        Eigen::Vector2d goal_pos(traj_matrix_(traj_matrix_.rows() - 1, 0), 
                                 traj_matrix_(traj_matrix_.rows() - 1, 1));
        Eigen::Vector2d current_pos_2d(x, y);
        double distance_to_goal = (current_pos_2d - goal_pos).norm();

        // 添加提前停靠逻辑 - 当距离终点很近时，不再执行MPC而是直接发布零速度
        double EARLY_STOP_THRESHOLD = 0.15; // 提前停止阈值
        
        if (distance_to_goal < EARLY_STOP_THRESHOLD) {
            // 距离终点很近时，提前停止MPC计算，直接发布零速度
            // auto velocity_msg = geometry_msgs::msg::Twist();
            // velocity_msg.linear.x = 0.0;
            // velocity_msg.linear.y = 0.0;
            // velocity_msg.linear.z = 0.0;
            // velocity_msg.angular.x = 0.0;
            // velocity_msg.angular.y = 0.0;
            // velocity_msg.angular.z = 0.0;
            
            // mpc_velocity_publisher_->publish(velocity_msg);
            
            // 发送零速度给底层控制（车体坐标系）
            sendVelocityCommand(Eigen::Vector2d::Zero(), yaw);
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Early stop activated: Distance to goal %.3f m < %.3f m, publishing zero velocity",
                               distance_to_goal, EARLY_STOP_THRESHOLD);
            
            // 更新到达状态
            goal_reached_ = true;
            prev_cmd_ = Eigen::Vector2d::Zero();
            // 清空轨迹，表示当前任务已完成
            traj_matrix_ = Eigen::MatrixXd(0, 4);
            last_traj_index_ = 0;
            return;
        }
        
        // 为全向轮小车准备MPC参考轨迹（从当前位置开始的未来轨迹）
        // 我们使用简化三状态模型：状态 [px, py, yaw]，控制输入为速度 [vx, vy, wz]
        Eigen::MatrixXd reference_traj(mpc_horizon_, 3);  // [px, py, yaw]

        // 计算当前最近点的切线方向，用于曲线跟踪优化
        double tangent_yaw = calculateTrajectoryTangent(target_index);
        
        // 填充参考轨迹（优先跟踪位置，yaw设为路径切线方向）
        for (int i = 0; i < mpc_horizon_; i++) {
            int ref_idx = std::min(target_index + i, static_cast<int>(traj_matrix_.rows() - 1));
            reference_traj(i, 0) = traj_matrix_(ref_idx, 0);  // px
            reference_traj(i, 1) = traj_matrix_(ref_idx, 1);  // py
            
            // 使用切线方向而不是固定为0，这有助于改善曲线跟踪
            if (ref_idx + 1 < static_cast<int>(traj_matrix_.rows())) {
                double dx = traj_matrix_(ref_idx + 1, 0) - traj_matrix_(ref_idx, 0);
                double dy = traj_matrix_(ref_idx + 1, 1) - traj_matrix_(ref_idx, 1);
                reference_traj(i, 2) = atan2(dy, dx);  // yaw - 切线方向
            } else {
                reference_traj(i, 2) = tangent_yaw;  // 使用当前切线方向
            }
        }

        // 构建当前状态向量 [x, y, yaw]
        Eigen::VectorXd current_state(3);
        current_state << x, y, yaw;

        // 设定系统矩阵A和B（简化三状态模型）
        Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
        // B矩阵：控制输入为 [vx, vy, wz]，积分到位置/yaw
        Eigen::Matrix<double, 3, 3> B = Eigen::Matrix<double, 3, 3>::Zero();
        B(0, 0) = dt_; // px_{k+1} = px_k + dt * vx
        B(1, 1) = dt_; // py_{k+1} = py_k + dt * vy
        B(2, 2) = dt_; // yaw_{k+1} = yaw_k + dt * wz

        // 调用MPC求解器（返回整个控制序列：N * n_controls）
        Eigen::VectorXd control_sequence = solveMPC(current_state, reference_traj, A, B);

        if (control_sequence.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "MPC returned empty control sequence");
            return;
        }

        // 取第一步的速度指令 [vx, vy, wz]
        Eigen::Vector3d u0 = control_sequence.segment(0, 3);
        Eigen::Vector2d target_vel(u0(0), u0(1));
        
        // 对目标速度做限幅和速率限制，避免突然的大速度跳变
        Eigen::Vector2d limited = target_vel;
        double speed = limited.norm();
        if (speed > max_lin_speed_) {
            limited *= (max_lin_speed_ / speed);
        }

        // 加速度限制（基于上一条命令与时间步长dt_）
        Eigen::Vector2d delta = limited - prev_cmd_;
        double max_delta = max_lin_accel_ * dt_;
        double delta_norm = delta.norm();
        if (delta_norm > max_delta && delta_norm > 1e-6) {
            limited = prev_cmd_ + delta * (max_delta / delta_norm);
        }

        // 发布限幅后的速度（地图坐标系）
        // auto velocity_msg = geometry_msgs::msg::Twist();
        // velocity_msg.linear.x = limited.x();
        // velocity_msg.linear.y = limited.y();
        // velocity_msg.linear.z = 0.0;
        // velocity_msg.angular.x = 0.0;
        // velocity_msg.angular.y = 0.0;
        // velocity_msg.angular.z = 0.0;

        // mpc_velocity_publisher_->publish(velocity_msg);

        // 发送给底层控制（自动进行坐标系转换：地图系 -> 车体系）
        sendVelocityCommand(limited, yaw);

        // 保存当前命令，以便下一次做速率限制
        prev_cmd_ = limited;
        
        // 调试输出
        Eigen::Vector2d vel_body = mapFrameToBodyFrame(limited, yaw);
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                     "Map frame: (%.3f, %.3f) m/s -> Body frame: (%.3f, %.3f) m/s (yaw: %.3f rad)",
                     limited.x(), limited.y(), vel_body.x(), vel_body.y(), yaw);
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                     "Published MPC velocity: (%.3f, %.3f)", 
                     prev_cmd_.x(), prev_cmd_.y());
    }
    
    // 新增：计算轨迹切线方向
    double calculateTrajectoryTangent(int idx) {
        if (traj_matrix_.rows() < 2) return 0.0;
        
        // 边界情况处理
        if (idx <= 0) idx = 1;
        if (idx >= static_cast<int>(traj_matrix_.rows())) idx = static_cast<int>(traj_matrix_.rows() - 1);
        
        // 计算当前位置附近轨迹的切线方向
        double dx = traj_matrix_(idx, 0) - traj_matrix_(idx-1, 0);
        double dy = traj_matrix_(idx, 1) - traj_matrix_(idx-1, 1);
        
        return atan2(dy, dx);
    }
    
    // MPC求解函数（通用动态尺寸）：state_dim = 3 (px,py,yaw)，control_dim = 3 (vx,vy,wz)
    Eigen::VectorXd solveMPC(const Eigen::VectorXd& current_state,
                             const Eigen::MatrixXd& reference_traj,
                             const Eigen::MatrixXd& A,
                             const Eigen::MatrixXd& B) {
        int N = mpc_horizon_;
        int n_states = static_cast<int>(A.rows());    // 3
        int n_controls = static_cast<int>(B.cols());  // 3

        // 如果轨迹矩阵为空，返回零控制序列
        if (reference_traj.rows() == 0) {
            return Eigen::VectorXd::Zero(N * n_controls);
        }

        // 构建增广系统矩阵 Phi 和 Psi
        Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(N * n_states, N * n_controls);
        Eigen::MatrixXd Psi = Eigen::MatrixXd::Zero(N * n_states, n_states);

        Eigen::MatrixXd power_A = Eigen::MatrixXd::Identity(n_states, n_states);
        for (int k = 0; k < N; k++) {
            Psi.block(k * n_states, 0, n_states, n_states) = power_A;

            for (int j = 0; j <= k; j++) {
                Eigen::MatrixXd Aj = Eigen::MatrixXd::Identity(n_states, n_states);
                // A^(k-j)
                for (int p = 0; p < (k - j); p++) Aj = Aj * A;
                Phi.block(k * n_states, j * n_controls, n_states, n_controls) = Aj * B;
            }

            power_A = power_A * A;
        }

        // 权重矩阵 Q, R
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n_states, n_states);
        Q(0, 0) = q_pos_;
        Q(1, 1) = q_pos_;
        if (n_states > 2) Q(2, 2) = q_yaw_;

        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(n_controls, n_controls) * r_ctrl_;
        if (n_controls >= 3) R(2, 2) = r_tau_; // yaw rate weight reuse

        // Q_bar, R_bar
        Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(N * n_states, N * n_states);
        for (int i = 0; i < N; ++i) Q_bar.block(i * n_states, i * n_states, n_states, n_states) = Q;

        Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(N * n_controls, N * n_controls);
        for (int i = 0; i < N; ++i) R_bar.block(i * n_controls, i * n_controls, n_controls, n_controls) = R;

        // Hessian P and gradient q
        Eigen::MatrixXd P_matrix = Phi.transpose() * Q_bar * Phi + R_bar;

        // 添加控制变化率约束（差分矩阵）
        if (N > 1 && r_dctrl_ > 1e-9) {
            Eigen::MatrixXd D = Eigen::MatrixXd::Zero((N - 1) * n_controls, N * n_controls);
            for (int i = 0; i < N - 1; ++i) {
                D.block(i * n_controls, i * n_controls, n_controls, n_controls) = -Eigen::MatrixXd::Identity(n_controls, n_controls);
                D.block(i * n_controls, (i + 1) * n_controls, n_controls, n_controls) = Eigen::MatrixXd::Identity(n_controls, n_controls);
            }
            P_matrix += r_dctrl_ * D.transpose() * D;
        }

        // 添加小的正则化项以确保数值稳定性
        P_matrix += Eigen::MatrixXd::Identity(N * n_controls, N * n_controls) * 1e-6;

        Eigen::VectorXd x0_replicated = Psi * current_state;

        // 参考向量
        Eigen::VectorXd ref_vec = Eigen::VectorXd::Zero(N * n_states);
        for (int i = 0; i < N; ++i) {
            Eigen::Index traj_row = std::min(static_cast<Eigen::Index>(i), reference_traj.rows() - 1);
            // reference_traj 每行是 [px, py, yaw]
            ref_vec.segment(i * n_states, n_states) = reference_traj.row(traj_row).transpose();
        }

        Eigen::VectorXd diff_vec = ref_vec - x0_replicated;
        Eigen::VectorXd q_vec = -2.0 * Phi.transpose() * Q_bar * diff_vec;

        int n_vars = N * n_controls;

        // 验证尺寸
        assert(P_matrix.rows() == P_matrix.cols());
        assert(P_matrix.rows() == n_vars);
        assert(q_vec.size() == n_vars);

        // 准备 OSQP 数据结构
        OSQPWorkspace *work;
        OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));
        OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));

        // 填充 data
        data->n = n_vars;
        data->m = 0; // 无约束的简化形式

        csc *P_osqp = eigen_to_csc(P_matrix);

        // 创建空的 A 矩阵（0 行）
        csc *A_osqp = (csc *)c_malloc(sizeof(csc));
        A_osqp->m = 0;
        A_osqp->n = n_vars;
        A_osqp->nz = -1;
        A_osqp->nzmax = 0;
        A_osqp->x = (double *)c_malloc(sizeof(double));
        A_osqp->i = (c_int *)c_malloc(sizeof(c_int));
        A_osqp->p = (c_int *)c_malloc((n_vars + 1) * sizeof(c_int));
        for (int i = 0; i <= n_vars; i++) A_osqp->p[i] = 0;

        data->P = P_osqp;
        data->q = q_vec.data();
        data->A = A_osqp;
        data->l = NULL;
        data->u = NULL;

        osqp_set_default_settings(settings);
        settings->verbose = false;
        settings->warm_start = true;
        settings->polish = true;
        settings->eps_abs = 1e-4;  // 适当放宽容差
        settings->eps_rel = 1e-4;

        int exitflag = osqp_setup(&work, data, settings);
        if (exitflag != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error setting up OSQP solver: %d", exitflag);
            if (P_osqp) csc_spfree(P_osqp);
            c_free(data);
            c_free(settings);
            return Eigen::VectorXd::Zero(n_vars);
        }

        osqp_solve(work);

        Eigen::VectorXd solution = Eigen::VectorXd::Zero(n_vars);
        if (work->solution && work->solution->x) {
            solution = Eigen::Map<Eigen::VectorXd>(work->solution->x, n_vars);
        } else {
            RCLCPP_ERROR(this->get_logger(), "OSQP solver failed to find solution");
        }

        // 清理
        osqp_cleanup(work);
        if (P_osqp) csc_spfree(P_osqp);
        if (A_osqp) {
            c_free(A_osqp->x);
            c_free(A_osqp->i);
            c_free(A_osqp->p);
            c_free(A_osqp);
        }
        c_free(data);
        c_free(settings);

        // 返回整个控制序列（N * n_controls）
        return solution;
    }
    
    // 状态预测函数（动态尺寸，A,B 可为任意尺寸匹配 current_state/controls）
    std::vector<Eigen::VectorXd> predictStates(const Eigen::VectorXd& initial_state,
                                               const Eigen::VectorXd& controls,
                                               const Eigen::MatrixXd& A,
                                               const Eigen::MatrixXd& B) {
        std::vector<Eigen::VectorXd> states;
        states.push_back(initial_state);

        Eigen::VectorXd current_state = initial_state;
        int n_controls = static_cast<int>(B.cols());

        for (int i = 0; i < mpc_horizon_ && i * n_controls + n_controls <= controls.size(); i++) {
            Eigen::VectorXd u = controls.segment(i * n_controls, n_controls);
            Eigen::VectorXd next_state = A * current_state + B * u;
            states.push_back(next_state);
            current_state = next_state;
        }

        return states;
    }
    
    
    // 将Eigen矩阵转换为osqp稀疏矩阵格式（仅保留上三角部分，用于对称矩阵）
    csc* eigen_to_csc(const Eigen::MatrixXd& mat) {
        int rows = mat.rows();
        int cols = mat.cols();
        int nnz = 0;
        
        // 统计上三角部分的非零元素
        for (int j = 0; j < cols; j++) {
            for (int i = 0; i <= j && i < rows; i++) {  // 仅统计i <= j的上三角部分
                if (std::abs(mat(i, j)) > 1e-9) {
                    nnz++;
                }
            }
        }
        
        // 分配内存
        csc *csc_mat = (csc*)c_malloc(sizeof(csc));
        csc_mat->m = rows;
        csc_mat->n = cols;
        csc_mat->nz = -1;  // CSC格式
        csc_mat->nzmax = nnz;
        
        csc_mat->x = (double*)c_malloc(nnz * sizeof(double));
        csc_mat->i = (c_int*)c_malloc(nnz * sizeof(c_int));
        csc_mat->p = (c_int*)c_malloc((cols + 1) * sizeof(c_int));
        
        // 填充数据（仅上三角）
        int idx = 0;
        csc_mat->p[0] = 0;
        
        for (int j = 0; j < cols; j++) {
            for (int i = 0; i <= j && i < rows; i++) {  // 仅处理上三角部分
                if (std::abs(mat(i, j)) > 1e-9) {
                    csc_mat->x[idx] = mat(i, j);
                    csc_mat->i[idx] = i;
                    idx++;
                }
            }
            csc_mat->p[j + 1] = idx;
        }
        
        return csc_mat;
    }
    
    
    
    void process_path(const nav_msgs::msg::Path::SharedPtr path_msg)
    {
        size_t num_points = path_msg->poses.size();
        path_matrix_ = Eigen::MatrixXd(num_points, 2);  
        
        for (size_t i = 0; i < num_points; ++i) {
            const auto& pose = path_msg->poses[i].pose;
            path_matrix_(i, 0) = pose.position.x;  // x坐标
            path_matrix_(i, 1) = pose.position.y;  // y坐标
        }
        
        RCLCPP_INFO(this->get_logger(), "更新路径矩阵，包含 %zu 个路径点", num_points);
        
        // 输出路径点信息用于调试
        if (num_points > 0) {
            if (num_points > 0) {
                RCLCPP_INFO(this->get_logger(), "第一个路径点: (%.3f, %.3f)", 
                            path_matrix_(0, 0), path_matrix_(0, 1));
                RCLCPP_INFO(this->get_logger(), "最后一个路径点: (%.3f, %.3f)", 
                            path_matrix_(num_points-1, 0), path_matrix_(num_points-1, 1));
            }
        }
        // 收到新的路径时，解除到达锁定（允许重新执行控制）
        goal_reached_ = false;
        // 重置轨迹索引，确保下一次从头开始查找
        last_traj_index_ = 0;
    }


    void process_traj(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr traj_msg)
    { 
        // 处理空轨迹
        size_t num_points = traj_msg->points.size();
        if (num_points == 0) {
            traj_matrix_ = Eigen::MatrixXd(0, 4);
            RCLCPP_WARN(this->get_logger(), "Received empty trajectory message.");
            // 清除上一次命令，解除到达标志
            prev_cmd_ = Eigen::Vector2d::Zero();
            // goal_reached_ = false;
            return;
        }

        traj_matrix_ = Eigen::MatrixXd::Zero(num_points, 4);

        for (size_t i = 0; i < num_points; ++i) {
            const auto &pt = traj_msg->points[i];

            // 默认值
            double px = 0.0, py = 0.0;
            double vx = 0.0, vy = 0.0;

            // transforms: 通常第0个transform包含位置
            if (!pt.transforms.empty()) {
                const auto &t = pt.transforms[0];
                px = t.translation.x;
                py = t.translation.y;
            } else {
                RCLCPP_DEBUG(this->get_logger(), "traj point %zu has no transforms", i);
            }

            // velocities: 通常第0个velocity包含线速度
            if (!pt.velocities.empty()) {
                const auto &v = pt.velocities[0].linear;
                vx = v.x;
                vy = v.y;
            } else {
                RCLCPP_DEBUG(this->get_logger(), "traj point %zu has no velocities", i);
            }

            traj_matrix_(i, 0) = px;
            traj_matrix_(i, 1) = py;
            traj_matrix_(i, 2) = vx;
            traj_matrix_(i, 3) = vy;
        }

        RCLCPP_INFO(this->get_logger(), "Updated trajectory matrix with %zu trajectory points", num_points);

        // 输出首尾轨迹点信息用于调试
        if (num_points > 0) {
            RCLCPP_INFO(this->get_logger(), "First trajectory point: (%.3f, %.3f) vel=(%.3f, %.3f)", 
                        traj_matrix_(0, 0), traj_matrix_(0, 1), traj_matrix_(0, 2), traj_matrix_(0, 3));
            RCLCPP_INFO(this->get_logger(), "Last trajectory point: (%.3f, %.3f) vel=(%.3f, %.3f)", 
                        traj_matrix_(num_points-1, 0), traj_matrix_(num_points-1, 1),
                        traj_matrix_(num_points-1, 2), traj_matrix_(num_points-1, 3));
        }
        // 收到新的轨迹，解除到达锁定（允许重新执行控制）
        goal_reached_ = false;
        // 重置轨迹索引，确保下一次从头开始查找
        last_traj_index_ = 0;
    }
    // TF2相关成员变量
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 路径点矩阵 (n×2矩阵，每行是一个路径点的[x, y])
    Eigen::MatrixXd path_matrix_;
    Eigen::MatrixXd traj_matrix_;
    Eigen::Matrix2d lidar_to_real; 

    // 变换参数
    double odom_to_map_x;
    double odom_to_map_y;
    double odom_to_map_yaw;
       
    // MPC相关参数
    int mpc_horizon_;
    double dt_;
    double q_pos_, q_yaw_, q_vel_, q_wyaw_, r_ctrl_, r_dctrl_, r_tau_;
    
    // 物理参数
    double mass_;      // 假设机器人质量
    double inertia_;   // 假设机器人绕z轴转动惯量

    // 速度/加速度限制与状态
    Eigen::Vector2d prev_cmd_; // 上一次发布的线速度 (vx, vy)
    double max_lin_speed_;     // 最大线速度 (m/s)
    double max_lin_accel_;     // 最大线加速度 (m/s^2)
    bool goal_reached_;

    // 预设大符坐标 
    double sign_x_;
    double sign_y_;
    
    // 预设巡逻点坐标
    double left_patrol_x_;
    double left_patrol_y_;
    double right_patrol_x_;
    double right_patrol_y_;
    double front_patrol_x_;
    double front_patrol_y_;
    double back_patrol_x_;
    double back_patrol_y_;

    // 预设补给坐标
    double origin_x_;
    double origin_y_;

    // 订阅者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr cmd_subscriber_;  

    // 添加成员变量用于跟踪上次索引
    int last_traj_index_ = 0;
    
    // 记录上一次命令的变量
    int last_cmd_ = -1;  // 初始化为-1，表示没有收到过命令
    
    // 发布者
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr mpc_velocity_publisher_;
    // 目标点发布器
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoGoGoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}