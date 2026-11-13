/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "common/common.h"
#include "small_point_lio/small_point_lio.h"
#include "voxelgrid_sampling/voxelgrid_sampling.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_point_lio/pch.h>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.h>

namespace small_point_lio {

    class SmallPointLioNode : public rclcpp::Node {
    private:
        std::unique_ptr<small_point_lio::SmallPointLio> small_point_lio;
        std::vector<common::Point> pointcloud;
        std::vector<Eigen::Vector3f> pointcloud_to_save;
        std::vector<Eigen::Vector3f> mapping_pointcloud_;
        nav_msgs::msg::Path mapping_path_;
        bool enable_mapping_visualization_;
        double mapping_visualization_leaf_size_;
        std::shared_ptr<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>> pointcloud_subsciber;
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subsciber;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>> acceleration_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> mapping_cloud_publisher_;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> mapping_path_publisher_;
        rclcpp::TimerBase::SharedPtr mapping_visualization_timer_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_trigger;
        common::Odometry last_odometry;

    public:
        explicit SmallPointLioNode(const rclcpp::NodeOptions &options);

    private:
        void pointcloud_callback(const livox_ros_driver2::msg::CustomMsg &msg);
        void imu_callback(const sensor_msgs::msg::Imu &msg);
        void publish_mapping_visualization();
        
    };

}// namespace small_point_lio
