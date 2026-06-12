/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <small_point_lio/pch.h>

namespace common {
    // Odometry- 里程计信息
    struct Odometry {
        double timestamp;                // 时间戳（秒）
        Eigen::Vector3d position;        // 位置（米）
        Eigen::Vector3d velocity;        // 速度（米/秒）
        Eigen::Quaterniond orientation;  // 姿态（四元数，弧度）
        Eigen::Vector3d angular_velocity;// 角速度（弧度/秒）
    };
    // ImuMsg- IMU传感器数据
    struct ImuMsg {
        double timestamp;                   // 时间戳（秒）
        Eigen::Vector3d linear_acceleration;// 线性加速度（g）
        Eigen::Vector3d angular_velocity;   // 角速度（弧度/秒）
    };
    // Point- 激光点云数据
    struct Point {
        double timestamp;        // 时间戳（秒）
        Eigen::Vector3f position;// 点位置（米）
    };

}// namespace common
