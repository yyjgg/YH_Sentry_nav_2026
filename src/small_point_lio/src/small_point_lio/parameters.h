/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <small_point_lio/pch.h>

namespace small_point_lio {

    class Parameters {
    public:
        // === 点云过滤参数 ===
        int point_filter_num;           // 点云稀疏化：每n个点取1个点，1表示不过滤
        float min_distance_squared;     // 最小距离平方（从min_distance计算而来），小于此值的点被过滤
        float max_distance_squared;     // 最大距离平方（从max_distance计算而来），大于此值的点被过滤
        bool space_downsample;          // 是否启用空间降采样（体素滤波）
        float space_downsample_leaf_size; // 空间降采样的体素栅格尺寸（米）

        // === IMU处理参数 ===
        Eigen::Vector3d gravity;        // 重力向量 [0.0, 0.0, -9.810] (m/s²)
        bool check_satu;                // 是否启用IMU数据饱和检查
        bool fix_gravity_direction;     // 是否使用前200个IMU数据修正重力方向
        double satu_acc;                // 加速度计饱和阈值（g），超过此值的数据被忽略
        double satu_gyro;               // 陀螺仪饱和阈值（rad/s），超过此值的数据被忽略
        double acc_norm;                // 加速度单位换算系数（1.0表示1g=9.81m/s²）

        // === 地图参数 ===
        double map_resolution;          // 体素地图的分辨率（米）
        size_t init_map_size;           // 初始化地图所需的最小点数

        // === 雷达-IMU外参 ===
        bool extrinsic_est_en;          // 是否在线估计雷达与IMU的相对位姿
        Eigen::Vector3d extrinsic_T;    // 雷达到IMU的平移外参 [x, y, z] (米)
        Eigen::Matrix3d extrinsic_R;    // 雷达到IMU的旋转外参（3x3旋转矩阵）

        // === 滤波器参数 ===
        
        // 观测噪声协方差（R矩阵）- 测量不确定性
        double laser_point_cov;         // 激光点观测噪声协方差
        double imu_meas_acc_cov;        // IMU加速度测量噪声协方差
        double imu_meas_omg_cov;        // IMU角速度测量噪声协方差
        
        // 过程噪声协方差（Q矩阵）- 系统模型不确定性
        double velocity_cov;           // 速度过程噪声协方差
        double omg_cov;                // 角速度过程噪声协方差
        double acceleration_cov;       // 加速度过程噪声协方差
        double bg_cov;                 // 陀螺仪零偏过程噪声协方差
        double ba_cov;                 // 加速度计零偏过程噪声协方差
        
        // 点云匹配参数
        double plane_thr;               // 平面拟合阈值，越小表示平面判断越严格
        double match_s;                 // 点面匹配阈值，越小表示匹配条件越严格

        // === 数据发布参数 ===
        bool publish_odometry_without_downsample = false; // 是否发布高频里程计（可能影响性能）

        void read_parameters(rclcpp::Node &node);
    };

}// namespace small_point_lio
