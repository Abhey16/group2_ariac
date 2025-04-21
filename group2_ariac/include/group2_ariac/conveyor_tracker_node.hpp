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
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class ConveyorTracker : public rclcpp::Node {
public:
    ConveyorTracker();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};
