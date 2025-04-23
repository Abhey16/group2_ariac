/**
 * @file start_competition_node.hpp
 * @author Wei-Li, Chen (wc2023@umd.com)
 * @brief Class definition for the ConveyorTracker class
 * @version 0.1
 * @date 2025-04-21
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <group2_msgs/msg/part.hpp>
#include <group2_msgs/msg/part_list.hpp>

class ConveyorTracker : public rclcpp::Node {
public:
    ConveyorTracker();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    geometry_msgs::msg::PointStamped projectPixelToWorld(int u, int v, float depth);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    rclcpp::Publisher<group2_msgs::msg::PartList>::SharedPtr detected_parts_pub_;
    group2_msgs::msg::PartList detections_msg_;

    rclcpp::TimerBase::SharedPtr publish_parts_timer_;
    void publish_parts_cb();

    bool first_detection_ = false;
    double update_timer_{};

    image_geometry::PinholeCameraModel camera_model_;
    bool camera_ready_;
    cv::Mat latest_depth_;
};
