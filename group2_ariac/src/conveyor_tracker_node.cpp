#include "conveyor_tracker_node.hpp"

ConveyorTracker::ConveyorTracker()
    : Node("conveyor_tracker_node")
{
    RCLCPP_INFO(this->get_logger(), "Conveyor Tracker Node Started");

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/ariac/sensors/ceiling_conveyor_camera/rgb_image", 10,
        std::bind(&ConveyorTracker::imageCallback, this, std::placeholders::_1));
}

void ConveyorTracker::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{

    cv::Mat image;
    try {
        image = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // parts define
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    cv::Scalar blue_lower(100, 100, 100), blue_upper(130, 255, 255);
    cv::Scalar purple_lower(130, 50, 50), purple_upper(160, 255, 255);

    cv::Mat blue_mask, purple_mask;
    cv::inRange(hsv, blue_lower, blue_upper, blue_mask);
    cv::inRange(hsv, purple_lower, purple_upper, purple_mask);

    // object detect
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& c : contours) {
        if (cv::contourArea(c) > 2000) {
            RCLCPP_INFO(this->get_logger(), "BLUE BATTERY detected in center.");

            double cam_x = -0.6;
            double cam_y = 3.63;
            double cam_z = 1.35;
            double obj_z = 0.875;
            double belt_speed_ = 0.345;

            double y = cam_y - (cam_z - obj_z);

            RCLCPP_INFO(this->get_logger(), "BLUE BATTERY at [%.3f, %.3f, %.3f]", cam_x, y, obj_z);

            // predic 1s, 2s later location
            for (int sec = 1; sec <= 2; ++sec) {
                double pred_y = y - belt_speed_ * sec;
                RCLCPP_INFO(this->get_logger(),
                "Prediction [%ds]: at [%.3f, %.3f, %.3f]",
                sec, cam_x, pred_y, obj_z);
            }
        }
    }

    cv::findContours(purple_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& c : contours) {
        if (cv::contourArea(c) > 2000) {
            RCLCPP_INFO(this->get_logger(), "PURPLE PUMP detected in center.");

            double cam_x = -0.6;
            double cam_y = 3.63;
            double cam_z = 1.35;
            double obj_z = 0.875;
            double belt_speed_ = 0.345;

            double y = cam_y - (cam_z - obj_z);

            RCLCPP_INFO(this->get_logger(), "PURPLE PUMP at [%.3f, %.3f, %.3f]", cam_x, y, obj_z);
            // predic 1s, 2s later location
            for (int sec = 1; sec <= 2; ++sec) {
                double pred_y = y - belt_speed_ * sec;
                RCLCPP_INFO(this->get_logger(),
                "Prediction [%ds]: at [%.3f, %.3f, %.3f]",
                sec, cam_x, pred_y, obj_z);
            }
        }
    }
}

// Main
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConveyorTracker>());
    rclcpp::shutdown();
    return 0;
}
