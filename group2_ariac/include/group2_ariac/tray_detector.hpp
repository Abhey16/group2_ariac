/**
 * @file tray_detector.hpp
 * @author Abhey Sharma (abheys16@umd.edu)
 * @brief Header file for the TrayDetector class that detects ArUco markers on trays from RGB images
 *        and estimates their poses relative to the camera and world frame using KDL and OpenCV.
 * @version 0.2
 * @date 2025-05-15
 * 
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/quaternion.hpp>
#include <Eigen/Geometry>
#include <kdl/frames.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include "ariac_msgs/msg/advanced_logical_camera_image.hpp"
#include "ariac_msgs/msg/kit_tray_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

/**
 * @class TrayDetector
 * @brief A ROS 2 node that subscribes to RGB camera images, detects ArUco markers, and computes 6D tray poses.
 */

class TrayDetector : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for TrayDetector.
     * @param node_name Name of the node.
     */
    TrayDetector(std::string node_name) : Node(node_name)
    {
        kt_right_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/ariac/sensors/rgb_camera_kts2/rgb_image",
                                                                           10,
                                                                           std::bind(&TrayDetector::kt_right_cb, this, std::placeholders::_1));

        kt_left_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/ariac/sensors/rgb_camera_kts1/rgb_image",
                                                                          10,
                                                                          std::bind(&TrayDetector::kt_left_cb, this, std::placeholders::_1));

        // Initialize timer to run part detection
        publish_trays_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrayDetector::publish_trays_cb, this));

        // Initialize publisher for detected parts
        detected_trays_pub_ = this->create_publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>("detected_trays", 10);
        // tray_msg_ = std::make_shared<ariac_msgs::msg::AdvancedLogicalCameraImage>();

        RCLCPP_INFO(this->get_logger(), "tray detector class created");
    }

    /**
     * @brief Callback for right RGB camera.
     * @param img Pointer to the received image message.
     */
    void kt_right_cb(sensor_msgs::msg::Image::ConstSharedPtr img);

    /**
     * @brief Callback for left RGB camera.
     * @param img Pointer to the received image message.
     */
    void kt_left_cb(sensor_msgs::msg::Image::ConstSharedPtr img);

    /**
     * @brief Callback for publishing detected trays
     */
    void publish_trays_cb();

    /**
     * @brief Detects ArUco markers in the given image and draws them.
     * @param input_img Input image in OpenCV format.
     * @param win_name Name of the OpenCV window for display.
     */
    void detect_aruco(cv::Mat input_img, std::string win_name);

    /**
     * @brief Estimates and prints the 6D pose of trays using ArUco markers.
     * @param input_img Input image in OpenCV format.
     * @param win_name Name of the OpenCV window for display.
     */
    void tray_pose(cv::Mat input_img, std::string win_name);

    /**
     * @brief Computes the world pose of the detected part using camera-to-world transform.
     * @param q Quaternion representing part orientation in camera frame.
     * @param x X position in camera frame.
     * @param y Y position in camera frame.
     * @param z Z position in camera frame.
     * @param kdl_camera_world KDL frame representing the camera pose in world frame.
     */
    geometry_msgs::msg::Pose part_world(const geometry_msgs::msg::Quaternion &q, const double &x, const double &y, const double &z, const KDL::Frame &kdl_camera_world);

    /**
     * @brief Converts a ROS geometry_msgs quaternion to a KDL rotation.
     * @param q Input quaternion.
     * @return KDL rotation object.
     */
    KDL::Rotation ros_quaternion_to_kdl_rotation(const geometry_msgs::msg::Quaternion &q);

    /**
     * @brief Converts roll, pitch, and yaw angles to a ROS quaternion.
     * @param roll Rotation about X axis.
     * @param pitch Rotation about Y axis.
     * @param yaw Rotation about Z axis.
     * @return ROS geometry_msgs quaternion.
     */
    geometry_msgs::msg::Quaternion euler_to_quaternions(const double &roll, const double &pitch, const double &yaw);

    /**
     * @brief Converts a KDL rotation to a ROS geometry_msgs quaternion.
     * @param rot KDL rotation object.
     * @return Equivalent ROS quaternion.
     */
    geometry_msgs::msg::Quaternion kdl_rotation_to_ros_quaternion(const KDL::Rotation &rot);

    /**
     * @brief Converts an OpenCV quaternion to a ROS geometry_msgs quaternion.
     * @param q OpenCV quaternion.
     * @return ROS quaternion.
     */
    geometry_msgs::msg::Quaternion cv_to_ros_quaternions(cv::Quatd q);

    /**
     * @brief Extracts roll, pitch, and yaw from a ROS geometry_msgs quaternion.
     * @param q Input ROS quaternion.
     * @return Tuple of (roll, pitch, yaw) angles in radians.
     */
    std::tuple<double, double, double> euler_from_quaternion(const geometry_msgs::msg::Quaternion &q);

    geometry_msgs::msg::Quaternion get_quat_msg(double x, double y, double z, double w);

private:
    /// Subscription to right RGB camera image
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr kt_right_sub_;

    /// Subscription to left RGB camera image
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr kt_left_sub_;

    /**
     * @brief Publisher for detected trays
     */
    rclcpp::Publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr detected_trays_pub_;

    /**
     * @brief Timer to publish detections
     */
    rclcpp::TimerBase::SharedPtr publish_trays_timer_;

    // Using LogicalCameraMessages to store and publish tray poses
    ariac_msgs::msg::AdvancedLogicalCameraImage tray_msg_;

    /// Latest RGB image from right camera
    cv::Mat kt_right_rgb{};

    /// Latest RGB image from left camera
    cv::Mat kt_left_rgb{};

    /// intrinsic parameters
    cv::Mat rgb_intrinsic = (cv::Mat_<double>(3, 3) << 343.49, 0.0, 320.5,
                             0.0, 343.49, 240.5,
                             0.0, 0.0, 1.0);

    /// distortion parameters
    cv::Mat rgb_dis = (cv::Mat::zeros(1, 5, CV_64F));
};