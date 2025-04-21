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

#include "conveyor_tracker_node.hpp"

ConveyorTracker::ConveyorTracker()
    : Node("conveyor_tracker_node")
{
    RCLCPP_INFO(this->get_logger(), "Conveyor Tracker Node Started");

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/ariac/sensors/ceiling_conveyor_camera/rgb_image",
        10,
        std::bind(&ConveyorTracker::imageCallback, this, std::placeholders::_1)
    );
}

void ConveyorTracker::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Convert image to OpenCV BGR format
    cv::Mat image;
    try {
        image = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Convert image to HSV color space
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // Color range for blue battery
    cv::Scalar blue_lower(100, 100, 100);
    cv::Scalar blue_upper(130, 255, 255);
    cv::Mat blue_mask;
    cv::inRange(hsv, blue_lower, blue_upper, blue_mask);

    // Color range for purple pump
    cv::Scalar purple_lower(130, 50, 50);
    cv::Scalar purple_upper(160, 255, 255);
    cv::Mat purple_mask;
    cv::inRange(hsv, purple_lower, purple_upper, purple_mask);

    // Check if anything was detected
    if (cv::countNonZero(blue_mask) > 2000) {
        RCLCPP_INFO(this->get_logger(), "Detected BLUE BATTERY on conveyor");
    }

    if (cv::countNonZero(purple_mask) > 2000) {
        RCLCPP_INFO(this->get_logger(), "Detected PURPLE PUMP on conveyor");
    }
}

// Main function to initialize the node and start spinning
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConveyorTracker>());
    rclcpp::shutdown();
    return 0;
}
