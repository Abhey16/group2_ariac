#include "tray_detector.hpp"

void TrayDetector::kt_right_cb(sensor_msgs::msg::Image::ConstSharedPtr img)
{   
    kt_right_rgb = cv_bridge::toCvShare(img,"bgr8")->image;

    cv::imshow("Display window - right", kt_right_rgb);

    TrayDetector::detect_aruco(kt_right_rgb, "right_tray");

    TrayDetector::tray_pose(kt_right_rgb, "right_tray_pose");

    cv::waitKey(1); // Wait for a keystroke in the window
 
}

void TrayDetector::kt_left_cb(sensor_msgs::msg::Image::ConstSharedPtr img)
{   
    kt_left_rgb = cv_bridge::toCvShare(img,"bgr8")->image;

    cv::imshow("Display window - left", kt_left_rgb);

    TrayDetector::detect_aruco(kt_left_rgb, "left_tray");

    TrayDetector::tray_pose(kt_left_rgb, "left_tray_pose");

    cv::waitKey(1); // Wait for a keystroke in the window
 
}

void TrayDetector::detect_aruco(cv::Mat input_img,std::string win_name)
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

void TrayDetector::tray_pose(cv::Mat input_img, std::string win_name)
{   
    cv::Mat imageCopy = input_img.clone();
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::aruco::detectMarkers(input_img, dictionary, corners, ids);

    if (ids.size() > 0)
    {    
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        
        // output parameters
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, 0.05, rgb_intrinsic, rgb_dis, rvecs, tvecs);


        for(size_t i=0; i<ids.size(); i++)
        {   

            cv::drawFrameAxes(imageCopy, rgb_intrinsic, rgb_dis, rvecs[i], tvecs[i], 0.1);

            cv::Quatd q = cv::Quatd::createFromRvec(rvecs[i]);

            RCLCPP_INFO(this->get_logger(), "##############################################");

            RCLCPP_INFO(this->get_logger(), "Tray %d : [%.4f %.4f %.4f] [%.4f %.4f %.4f %.4f]", ids[i],tvecs[i][0], tvecs[i][1], tvecs[i][2],q.w,q.x,q.y,q.z );

            RCLCPP_INFO(this->get_logger(), "##############################################");
        }
    }

    cv::imshow(win_name, imageCopy);
}

// Main function to initialize ROS node and start spinning
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // Initialize ROS with command-line arguments

    // Create a RetrieveOrders node instance
    auto node = std::make_shared<TrayDetector>("tray_detector");

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);

    executor.spin();  // This will start the execution

    // Shutdown ROS when done
    rclcpp::shutdown();

    return 0;
}
