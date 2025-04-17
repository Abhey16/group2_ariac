#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/quaternion.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class BinPartsDetector : public rclcpp::Node
{
public:
    BinPartsDetector(std::string node_name) : Node(node_name)
    {
        bin_left_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/ariac/sensors/rgb_left_bins/rgb_image",
                                                                           10,
                                                                           std::bind(&BinPartsDetector::bin_left_cb, this, std::placeholders::_1));

        bin_right_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/ariac/sensors/rgb_right_bins/rgb_image",
                                                                            10,
                                                                            std::bind(&BinPartsDetector::bin_right_cb, this, std::placeholders::_1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Toggle camera views
        enable_left_camera_ = false;
        enable_right_camera_ = false;
        roi_debug_ = false;
        debug_bins_ = false;

        for (const auto &color : colors)
        {
            for (const auto &type : types)
            {
                part_poses_[color][type] = {};
                centered_part_poses_[color][type] = {};
            }
        }

        RCLCPP_INFO(this->get_logger(), "Bin Parts Detector Node Initialized");

        if (!load_part_templates())
        {
            RCLCPP_FATAL(this->get_logger(), "Failed To Load Part Templates. Shutting Down Bin Parts Detector Node.");
            rclcpp::shutdown();
        }
    }

private:
    void bin_left_cb(sensor_msgs::msg::Image::ConstSharedPtr img);
    void bin_right_cb(sensor_msgs::msg::Image::ConstSharedPtr img);
    void get_bin_parts(const int &bin_number);
    void find_parts(const cv::Mat &img);
    bool load_part_templates();
    std::vector<cv::Rect> non_max_suppression(const std::vector<cv::Rect> &boxes, float overlapThreshold = 0.3);
    void match_template(const cv::Mat &imgMask, const std::string &color, const std::string &type);

    struct PartMsg
    {
        std::string color;
        std::string type;
    };

    struct PartInstance {
        std::string color;
        std::string type;
        int bin_number;
        geometry_msgs::msg::Point position;
        geometry_msgs::msg::Quaternion orientation;  // Placeholder for future use
    };

    std::vector<PartInstance> detected_parts_;


    std::map<int, PartMsg> output_by_slot();
    void print_slot_assignments(int bin_number);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr bin_left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr bin_right_sub_;

    cv::Mat bin_left_rgb_;
    cv::Mat bin_right_rgb_;
    cv::Mat sensor_template_;
    cv::Mat regulator_template_;
    cv::Mat battery_template_;
    cv::Mat pump_template_;

    std::map<std::string, std::map<std::string, std::vector<cv::Rect>>> part_poses_;
    std::map<std::string, std::map<std::string, std::vector<cv::Point>>> centered_part_poses_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<std::string> colors = {"red", "green", "blue", "orange", "purple"};
    std::vector<std::string> types = {"battery", "pump", "sensor", "regulator"};

    bool enable_left_camera_;
    bool enable_right_camera_;
    bool roi_debug_;
    bool debug_bins_;

    std::map<std::pair<int, int>, int> slot_mapping_ =
        {
            {{1, 1}, 1}, {{1, 2}, 2}, {{1, 3}, 3}, {{2, 1}, 4}, {{2, 2}, 5}, {{2, 3}, 6}, {{3, 1}, 7}, {{3, 2}, 8}, {{3, 3}, 9}};

    // intrinsic parameters
    cv::Mat rgb_intrinsic_ = (cv::Mat_<double>(3, 3) << 343.49, 0.0, 320.5,
                              0.0, 343.49, 240.5,
                              0.0, 0.0, 1.0);

    // distortion parameters
    cv::Mat rgb_dis_ = (cv::Mat::zeros(1, 5, CV_64F));

    geometry_msgs::msg::Point pixelToWorldPoint(
        const cv::Point2f &pixel,
        const std::string &camera_frame,
        double depth);

    std::map<std::string, std::map<std::string, std::vector<geometry_msgs::msg::Point>>> world_part_poses_;

    void print_all_detected_parts();

}; // class BinPartsDetector