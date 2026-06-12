/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "esekf.h"
#include "parameters.h"
#include "small_ivox.h"
#include <small_point_lio/pch.h>

namespace small_point_lio {

    class Estimator {
    public:
        Parameters *parameters = nullptr;                    // 算法参数配置指针
        Eigen::Vector3f point_lidar_frame;                   // 当前处理的激光点在激光雷达坐标系下的坐标
        Eigen::Vector3f point_odom_frame;                    // 当前处理的激光点在世界坐标系（里程计坐标系）下的坐标
        std::vector<Eigen::Vector3f> nearest_points;         // 最近邻点集合，用于平面拟合
        std::shared_ptr<SmallIVox> ivox = nullptr;           // 增量体素地图指针，用于存储和管理地图点云
        esekf kf;                                            // 误差状态卡尔曼滤波器（Error State Kalman Filter）实例
        Eigen::Vector3d angular_velocity, linear_acceleration; // 当前IMU测量的角速度和线性加速度
        Eigen::Vector3d Lidar_T_wrt_IMU;                    // 雷达到IMU的平移外参
        Eigen::Matrix3d Lidar_R_wrt_IMU;                    // 雷达到IMU的旋转外参
        double G_m_s2 = 9.81;                               // 重力加速度常数（m/s²）

        Estimator();                                        // 构造函数

        void reset();                                       // 重置估计器状态

        Eigen::Matrix<state::value_type, state::DIM, state::DIM> process_noise_cov(); // 计算过程噪声协方差矩阵Q

        Eigen::Matrix<state::value_type, state::DIM, 1> f_x(const state &s); // 状态转移函数（运动模型）

        Eigen::Matrix<state::value_type, state::DIM, state::DIM> df_dx(const state &s); // 状态转移函数的雅可比矩阵

        void h_point(const state &s, const Eigen::Matrix<state::value_type, 3, 3> &cov_p, const Eigen::Matrix<state::value_type, 3, 3> &cov_R, point_measurement_result &measurement_result); // 激光点云观测模型

        void h_imu(const state &s, imu_measurement_result &ekfom_data); // IMU观测模型
    };

}// namespace small_point_lio
