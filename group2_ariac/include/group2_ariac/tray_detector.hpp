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

class TrayDetector : public rclcpp::Node
{
public:
    TrayDetector(std::string node_name) : Node(node_name)
    {
        kt_right_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/ariac/sensors/rgb_camera_kts2/rgb_image",
                                                                           10,
                                                                           std::bind(&TrayDetector::kt_right_cb, this, std::placeholders::_1));

        kt_left_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/ariac/sensors/rgb_camera_kts1/rgb_image",
                                                                          10,
                                                                          std::bind(&TrayDetector::kt_left_cb, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "tray detector class created");
    }

    void kt_right_cb(sensor_msgs::msg::Image::ConstSharedPtr img);
    void kt_left_cb(sensor_msgs::msg::Image::ConstSharedPtr img);
    void detect_aruco(cv::Mat input_img, std::string win_name);
    void tray_pose(cv::Mat input_img, std::string win_name);

    void part_world(const geometry_msgs::msg::Quaternion &q, const double &x, const double &y, const double &z, const KDL::Frame &kdl_camera_world);
    KDL::Rotation ros_quaternion_to_kdl_rotation(const geometry_msgs::msg::Quaternion &q);
    geometry_msgs::msg::Quaternion euler_to_quaternions(const double &roll, const double &pitch, const double &yaw);
    geometry_msgs::msg::Quaternion kdl_rotation_to_ros_quaternion(const KDL::Rotation &rot);
    geometry_msgs::msg::Quaternion cv_to_ros_quaternions(cv::Quatd q);
    std::tuple<double, double, double> euler_from_quaternion(const geometry_msgs::msg::Quaternion &q);

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr kt_right_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr kt_left_sub_;

    cv::Mat kt_right_rgb{};
    cv::Mat kt_left_rgb{};

    // intrinsic parameters
    cv::Mat rgb_intrinsic = (cv::Mat_<double>(3, 3) << 343.49, 0.0, 320.5,
                             0.0, 343.49, 240.5,
                             0.0, 0.0, 1.0);

    // distortion parameters
    cv::Mat rgb_dis = (cv::Mat::zeros(1, 5, CV_64F));
};