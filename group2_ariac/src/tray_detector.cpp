#include "tray_detector.hpp"

void TrayDectector::kt_right_cb(sensor_msgs::msg::Image::ConstSharedPtr img)
{   
    kt_right_rgb = cv_bridge::toCvShare(img,"bgr8")->image;

    cv::imshow("Display window", kt_right_rgb);

    int k = cv::waitKey(1); // Wait for a keystroke in the window
 
}

// Main function to initialize ROS node and start spinning
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // Initialize ROS with command-line arguments

    // Create a RetrieveOrders node instance
    auto node = std::make_shared<TrayDectector>("tray_detector");

    // Start spinning the node to handle callbacks and events
    rclcpp::spin(node);
    
    // Shutdown ROS when done
    rclcpp::shutdown();

    return 0;
}
