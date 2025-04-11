#include "tray_detector.hpp"

void TrayDectector::kt_right_cb(sensor_msgs::msg::Image::ConstSharedPtr img)
{   
    kt_right_rgb = cv_bridge::toCvShare(img,"bgr8")->image;

    cv::imshow("Display window - right", kt_right_rgb);

    TrayDectector::detect_aruco(kt_right_rgb, "right_tray");

    int k = cv::waitKey(1); // Wait for a keystroke in the window
 
}

void TrayDectector::kt_left_cb(sensor_msgs::msg::Image::ConstSharedPtr img)
{   
    kt_left_rgb = cv_bridge::toCvShare(img,"bgr8")->image;

    cv::imshow("Display window - left", kt_left_rgb);

    TrayDectector::detect_aruco(kt_left_rgb, "left_tray");

    int k = cv::waitKey(1); // Wait for a keystroke in the window
 
}

void TrayDectector::detect_aruco(cv::Mat input_img,std::string win_name)
{   
    cv::Mat imageCopy = input_img.clone();
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::aruco::detectMarkers(input_img, dictionary, corners, ids);

    // if at least one marker detected
    if (ids.size() > 0)
    {
    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
    }

    cv::imshow(win_name, imageCopy);
}

// Main function to initialize ROS node and start spinning
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // Initialize ROS with command-line arguments

    // Create a RetrieveOrders node instance
    auto node = std::make_shared<TrayDectector>("tray_detector");

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);

    executor.spin();  // This will start the execution

    // Shutdown ROS when done
    rclcpp::shutdown();

    return 0;
}
