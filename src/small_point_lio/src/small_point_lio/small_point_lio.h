/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "../common/common.h"
#include "estimator.h"
#include "parameters.h"
#include "preprocess.h"
#include <small_point_lio/pch.h>

namespace small_point_lio {

    class SmallPointLio {
    private:
        Parameters parameters;     // 参数配置容器
        Preprocess preprocess;     // 数据预处理模块
        Estimator estimator;       // 实现ESKF算法，进行状态估计和地图维护
        double time_update_last = 0.0, time_predict_last = 0.0, time_current = 0.0;// imu数据采集时间，点云数据采集时间，KF系统时间
        std::vector<Eigen::Vector3f> pointcloud_odom_frame;
        std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> pointcloud_callback;
        std::function<void(const common::Odometry &odometry)> odometry_callback;
        bool is_init = false;
        size_t process_count_ = 0;
    public:
        Eigen::Matrix<double, 30, 30> Q;

        explicit SmallPointLio(rclcpp::Node &node);

        void reset();

        void on_point_cloud_callback(const std::vector<common::Point> &pointcloud);

        void on_imu_callback(const common::ImuMsg &imu_msg);

        void handle_once();

        void set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &pointcloud_callback);

        void set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback);

        Eigen::Matrix3d R_lidar_to_base_link;  // 雷达到底盘的旋转变化
        Eigen::Vector3d T_lidar_to_base_link;  // 雷达到底盘的平移变化

    private:
        void publish_odometry(double timestamp);
    };

}// namespace small_point_lio
