/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio.h"

namespace small_point_lio {

    SmallPointLio::SmallPointLio(rclcpp::Node &node) {
        // init param
        parameters.read_parameters(node);
        preprocess.parameters = &parameters;
        estimator.parameters = &parameters;
        estimator.Lidar_T_wrt_IMU = parameters.extrinsic_T;
        estimator.Lidar_R_wrt_IMU = parameters.extrinsic_R;
        if (parameters.extrinsic_est_en) {
            estimator.kf.x.offset_T_L_I = parameters.extrinsic_T;
            estimator.kf.x.offset_R_L_I = parameters.extrinsic_R;
        }
        Q = estimator.process_noise_cov();
        estimator.G_m_s2 = parameters.gravity.norm();
        R_lidar_to_base_link << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        T_lidar_to_base_link << 0,0,0.3;
        // init data
        reset();
    }

    void SmallPointLio::reset() {
        preprocess.reset();
        estimator.reset();
        is_init = false;
    }

    void SmallPointLio::on_point_cloud_callback(const std::vector<common::Point> &pointcloud) {
        preprocess.on_point_cloud_callback(pointcloud);
    }

    void SmallPointLio::on_imu_callback(const common::ImuMsg &imu_msg) {
        preprocess.on_imu_callback(imu_msg);
    }

    void SmallPointLio::handle_once() {
       
        // === 预测阶段 ===
        if (!is_init) {
             if ((!preprocess.point_deque.empty() || !preprocess.imu_deque.empty()) && preprocess.point_deque.size() >= parameters.init_map_size && (!parameters.fix_gravity_direction || preprocess.imu_deque.size() >= 200)) {
                // 初始化地图和重力方向
                for (const auto &point : preprocess.point_deque) {
                    estimator.ivox->add_point(point.position);  // 填入ivox
                }
                if (parameters.fix_gravity_direction) {
                    estimator.kf.x.gravity = Eigen::Vector3d::Zero();
                    for (const auto &imu_msg : preprocess.imu_deque) {
                        estimator.kf.x.gravity += imu_msg.linear_acceleration;
                    }
                    double scale = -parameters.gravity.norm() / estimator.kf.x.gravity.norm();
                    estimator.kf.x.gravity *= scale;
                }
                // init time
                if (preprocess.point_deque.empty()) {
                    time_current = preprocess.imu_deque.back().timestamp;
                } else if (preprocess.imu_deque.empty()) {
                    time_current = preprocess.point_deque.back().timestamp;
                } else {
                    time_current = std::max(preprocess.point_deque.back().timestamp, preprocess.imu_deque.back().timestamp);
                }
                time_predict_last = time_current;
                time_update_last = time_current;
                preprocess.point_deque.clear();
                preprocess.dense_point_deque.clear();
                preprocess.imu_deque.clear();
                is_init = true;
            }
            return;
        }
        // 判断是否需要发布里程计，因为我们希望保持正常的里程计发布频率
        bool is_publish_odometry =
            !preprocess.imu_deque.empty() &&
            !preprocess.point_deque.empty() &&
             preprocess.point_deque.front().timestamp < preprocess.imu_deque.back().timestamp &&
             preprocess.point_deque.back().timestamp > preprocess.imu_deque.front().timestamp;
        
        while (!preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty()) {
            const common::Point &point_lidar_frame = preprocess.point_deque.front();
            const common::Point &dense_point_lidar_frame = preprocess.dense_point_deque.front();
            const common::ImuMsg &imu_msg = preprocess.imu_deque.front();
            // 稠密点云数据最早到达，将稠密点云从激光坐标系转换到世界坐标系，存入 pointcloud_odom_frame用于后续建图发布
            if (dense_point_lidar_frame.timestamp < point_lidar_frame.timestamp && dense_point_lidar_frame.timestamp < imu_msg.timestamp) {
                // 收集lidar_odom坐标系点云
                Eigen::Vector3d dense_point_imu_frame;
                if (parameters.extrinsic_est_en) {
                    dense_point_imu_frame = estimator.kf.x.offset_R_L_I * dense_point_lidar_frame.position.cast<double>() + estimator.kf.x.offset_T_L_I;
                } else {
                    dense_point_imu_frame = estimator.Lidar_R_wrt_IMU * dense_point_lidar_frame.position.cast<double>() + estimator.Lidar_T_wrt_IMU;
                }
                pointcloud_odom_frame.emplace_back((R_lidar_to_base_link * (estimator.kf.x.rotation * dense_point_imu_frame + estimator.kf.x.position)
                    + T_lidar_to_base_link).cast<float>());
        
                preprocess.dense_point_deque.pop_front();
            } 
            // 稀疏点云数据最早到达
            else if (point_lidar_frame.timestamp < imu_msg.timestamp) {
                // 点云更新
                time_current = point_lidar_frame.timestamp;
                // 预测
                double dt = time_current - time_predict_last;
                if (dt > 0) {
                    estimator.kf.predict_state(dt);
                    time_predict_last = time_current;
                }
                
                // === 更新阶段 ===
                estimator.point_lidar_frame = point_lidar_frame.position;
                estimator.kf.update_iterated_point();

                if (parameters.publish_odometry_without_downsample) {
                    publish_odometry(time_current);
                }
                
                // === 地图维护阶段 ===
                estimator.ivox->add_point(estimator.point_odom_frame); // 将转换坐标系后的点填入ivox用于匹配
                
                preprocess.point_deque.pop_front();
                
            } 
            else {
                // IMU更新
                time_current = imu_msg.timestamp;

                // 预测
                double dt = time_current - time_predict_last;
                if (dt > 0) {
                    estimator.kf.predict_state(dt);
                    time_predict_last = time_current;
                }

                // === 更新阶段 ===
                estimator.angular_velocity = imu_msg.angular_velocity;
                estimator.linear_acceleration = imu_msg.linear_acceleration;
                double dt_cov = time_current - time_update_last;
                time_update_last = time_current;
                estimator.kf.predict_prop_cov(dt_cov, Q);
                estimator.kf.update_iterated_imu();
                preprocess.imu_deque.pop_front();
            }
        }

        if (!is_publish_odometry) {
            return;
        }

        // 发布里程计和点云
        if (!parameters.publish_odometry_without_downsample) {
            publish_odometry(time_current);
        }
        if (!pointcloud_odom_frame.empty()) {
            if (pointcloud_callback) {
                pointcloud_callback(pointcloud_odom_frame);
            }
            pointcloud_odom_frame.clear();

        }
    }

    void SmallPointLio::set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &pointcloud_callback) {
        this->pointcloud_callback = pointcloud_callback;
    }

    void SmallPointLio::set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback) {
        this->odometry_callback = odometry_callback;
    }

    void SmallPointLio::publish_odometry(double timestamp) {
        if (odometry_callback) {
            common::Odometry odometry;
            odometry.timestamp = timestamp;
            // odometry.position = R_lidar_to_base_link * estimator.kf.x.position + T_lidar_to_base_link;
            // odometry.velocity = R_lidar_to_base_link * estimator.kf.x.velocity;
            // Eigen::Quaterniond q_lidar_to_base_link(R_lidar_to_base_link);
            // odometry.orientation = q_lidar_to_base_link * estimator.kf.x.rotation;
            // odometry.angular_velocity = R_lidar_to_base_link * estimator.kf.x.omg;
            odometry.position =  estimator.kf.x.position;
            odometry.velocity =  estimator.kf.x.velocity;
            odometry.orientation = estimator.kf.x.rotation;
            odometry.angular_velocity = estimator.kf.x.omg;
            odometry_callback(odometry);
        }
    }

}// namespace small_point_lio
