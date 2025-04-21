/**
 * @file bin_parts_detector.hpp
 * @author Rey Roque-Perez (reyroque@umd.edu)
 * @brief Class definition for the BinPartsDetector class
 * @version 0.1
 * @date 2025-04-20
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/quaternion.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "group2_msgs/msg/part.hpp"
#include "group2_msgs/msg/part_list.hpp"

#include <string>

/**
 * @brief Class that detects and publishes parts found in ARIAC bins
 *
 */
class BinPartsDetector : public rclcpp::Node
{
public:
    BinPartsDetector(std::string node_name) : Node(node_name)
    {
        // Subscriber for left bin camera
        bin_left_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/ariac/sensors/rgb_left_bins/rgb_image",
                                                                           10,
                                                                           std::bind(&BinPartsDetector::bin_left_cb, this, std::placeholders::_1));

        // Subscriber for right bin camera
        bin_right_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/ariac/sensors/rgb_right_bins/rgb_image",
                                                                            10,
                                                                            std::bind(&BinPartsDetector::bin_right_cb, this, std::placeholders::_1));

        // Initialize timer to run part detection
        detect_parts_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&BinPartsDetector::detect_parts_cb, this));

        // Initialize publisher for detected parts
        detected_parts_pub_ = this->create_publisher<group2_msgs::msg::PartList>("detected_bin_parts", 10);

        // Broadcast transforms from camera to world
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        broadcast_camera_transforms();
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initializing coordinates map for each part color/type combination
        for (const auto &color : colors_)
        {
            for (const auto &type : types_)
            {
                part_roi_coordinates_[color][type] = {};
            }
        }

        RCLCPP_INFO(this->get_logger(), "Bin Parts Detector Node Initialized");

        // Load part templates for part type matching
        if (!load_part_templates())
        {
            RCLCPP_FATAL(this->get_logger(), "Failed To Load Part Templates. Shutting Down Bin Parts Detector Node.");
            rclcpp::shutdown();
        }
    }

private:
    /**
     * @brief Callback that reads the left camera image
     *
     * @param img [sensor_msgs::msg::Image::ConstSharedPtr] Camera image
     */
    void bin_left_cb(sensor_msgs::msg::Image::ConstSharedPtr img);

    /**
     * @brief Callback that reads the right camera image
     *
     * @param img [sensor_msgs::msg::Image::ConstSharedPtr] Camera image
     */
    void bin_right_cb(sensor_msgs::msg::Image::ConstSharedPtr img);

    /**
     * @brief Callback for running part detection on received images
     */
    void detect_parts_cb();

    /**
     * @brief Performs detection of all parts inside a bin
     *
     * @param bin_number [const int &] Bin to perform part detection on
     */
    void get_bin_parts(const int &bin_number);

    /**
     * @brief Finds parts in the given mage
     *
     * @param img [const cv::Mat &] CV Image
     */
    void find_parts(const cv::Mat &img);

    /**
     * @brief Loads templates for part matching
     *
     * @return true|false
     */
    bool load_part_templates();

    /**
     * @brief Non max suppresion method used to filter multiple detections
     *        Over-simplified aproach based on python's imutil module
     *
     * @param boxes [const std::vector<cv::Rect> &] Rectangle representing a detection
     * @param overlap_threshold [float] Threshold for repeat detections
     * @return std::vector<cv::Rect>
     */
    std::vector<cv::Rect> non_max_suppression(const std::vector<cv::Rect> &boxes, float overlap_threshold = 0.3);

    /**
     * @brief Match a detected part to a part type using a template
     *
     * @param img_mask [const cv::Mat &] Masked image to process
     * @param color [const std::string &] Color of the part being processed
     * @param type [const std::string &] Type to verify
     */
    void match_template(const cv::Mat &img_mask, const std::string &color, const std::string &type);

    /**
     * @brief Stores data representing a Part
     *
     */
    struct Part
    {
        std::string color;
        std::string type;
        int bin_number;
        geometry_msgs::msg::Point position;
        geometry_msgs::msg::Quaternion orientation;
    };

    /**
     * @brief Stores Parts
     */
    std::vector<Part> detected_parts_;

    /**
     * @brief Subcribers for left camera
     */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr bin_left_sub_;

    /**
     * @brief Subcribers for right camera
     */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr bin_right_sub_;

    /**
     * @brief Timer to run part detection
     */
    rclcpp::TimerBase::SharedPtr detect_parts_timer_;

    /**
     * @brief Publisher for detected parts
     */
    rclcpp::Publisher<group2_msgs::msg::PartList>::SharedPtr detected_parts_pub_;

    /**
     * @brief Broadcast transforms from camera to world
     */
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    /**
     * @brief Broadcast transforms from camera to world
     */
    void broadcast_camera_transforms();

    /**
     * @brief Stores rgb formatted image of left camera image
     */
    cv::Mat bin_left_rgb_;

    /**
     * @brief Stores rgb formatted image of right camera image
     */
    cv::Mat bin_right_rgb_;

    /**
     * @brief Stores sensor template used for masking
     */
    cv::Mat sensor_template_;

    /**
     * @brief Stores regulator template used for masking
     */
    cv::Mat regulator_template_;

    /**
     * @brief Stores battery template used for masking
     */
    cv::Mat battery_template_;

    /**
     * @brief Stores pump template used for masking
     */
    cv::Mat pump_template_;

    /**
     * @brief Stores parts and their coordinates in an roi cropped image
     */
    std::map<std::string, std::map<std::string, std::vector<cv::Point>>> part_roi_coordinates_;

    /**
     * @brief Transfrom buffer
     */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    /**
     * @brief Transform listener
     */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /**
     * @brief Possible part colors
     */
    std::vector<std::string> colors_ = {"red", "green", "blue", "orange", "purple"};

    /**
     * @brief Possible part types
     */
    std::vector<std::string> types_ = {"battery", "pump", "sensor", "regulator"};

    /**
     * @brief Mapping a 3x3 grid to bin slots ranging from 1-9
     */
    std::map<std::pair<int, int>, int> slot_mapping_ =
        {
            {{1, 1}, 1}, {{1, 2}, 2}, {{1, 3}, 3}, {{2, 1}, 4}, {{2, 2}, 5}, {{2, 3}, 6}, {{3, 1}, 7}, {{3, 2}, 8}, {{3, 3}, 9}};

    /**
     * @brief Camera intrinsic parameters
     */
    cv::Mat rgb_intrinsic_ = (cv::Mat_<double>(3, 3) << 343.49636753580074, 0.0, 320.5,
                              0.0, 343.49636753580074, 240.5,
                              0.0, 0.0, 1.0);
    /**
     * @brief Camera distortion parameters
     */
    cv::Mat rgb_dis_ = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);

    /**
     * @brief Convert camera image pixel coordinate to a camera frame pose
     *
     * @param pixel [const cv::Point2f &] Center pixel location of a part detection
     * @param optical_frame [const std::string &] Camera optical frame name
     * @param depth [double] Distance between camera and parts/bin surface
     * @return geometry_msgs::msg::PointStamped
     */
    geometry_msgs::msg::PointStamped pixel_to_optical_frame(const cv::Point2f &pixel,
                                                            const std::string &optical_frame,
                                                            double depth);

    /**
     * @brief Stores part poses in world frame
     */
    std::map<std::string, std::map<std::string, std::vector<geometry_msgs::msg::Point>>> world_part_poses_;

    /**
     * @brief Logs detected parts for debugging
     */
    void print_detected_parts();

}; // class BinPartsDetector