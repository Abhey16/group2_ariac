#include "tray_detector.hpp"

void TrayDectector::kt_right_cb(sensor_msgs::msg::Image::ConstSharedPtr img)
{   
    kt_right_rgb = cv_bridge::toCvShare(img,"bgr8")->image;

    cv::imshow("Display window - right", kt_right_rgb);

    int k = cv::waitKey(1); // Wait for a keystroke in the window
 
}

void TrayDectector::kt_left_cb(sensor_msgs::msg::Image::ConstSharedPtr img)
{   
    kt_left_rgb = cv_bridge::toCvShare(img,"bgr8")->image;

    cv::imshow("Display window - left", kt_left_rgb);

    int k = cv::waitKey(1); // Wait for a keystroke in the window
 
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
