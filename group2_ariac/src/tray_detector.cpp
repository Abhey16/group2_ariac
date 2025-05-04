/**
 * @file tray_detector.cpp
 * @author Abhey Sharma (abheys16@umd.edu)
 * @brief tray_detector.cpp implement the function defined in the headr file

 */

#include "tray_detector.hpp"

// Callback for right RGB camera image
void TrayDetector::kt_right_cb(sensor_msgs::msg::Image::ConstSharedPtr img)
{
    // Convert ROS image message to OpenCV image (BGR8 encoding)
    kt_right_rgb = cv_bridge::toCvShare(img, "bgr8")->image;

    // // Display image output of rgb camera
    // cv::imshow("Display window - right", kt_right_rgb);

    // TrayDetector::detect_aruco(kt_right_rgb, "right_tray");

    // Call pose estimation function for the right tray
    TrayDetector::tray_pose(kt_right_rgb, "kts2_tray_pose");

    // Required for OpenCV window to update
    cv::waitKey(1);
}

// Callback for left RGB camera image
void TrayDetector::kt_left_cb(sensor_msgs::msg::Image::ConstSharedPtr img)
{
    // Convert ROS image message to OpenCV image (BGR8 encoding)
    kt_left_rgb = cv_bridge::toCvShare(img, "bgr8")->image;

    // // Display image output of rgb camera
    // cv::imshow("Display window - left", kt_left_rgb);

    // TrayDetector::detect_aruco(kt_left_rgb, "left_tray");

    // Call pose estimation function for the left tray
    TrayDetector::tray_pose(kt_left_rgb, "kts1_tray_pose");

    // Required for OpenCV window to update
    cv::waitKey(1); // Wait for a keystroke in the window
}

// Detects and highlights ArUco markers in the input image
void TrayDetector::detect_aruco(cv::Mat input_img, std::string win_name)
{
    cv::Mat imageCopy = input_img.clone();
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    // Use 4x4_1000 ArUco dictionary for detection
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
    cv::aruco::detectMarkers(input_img, dictionary, corners, ids);

    // if at least one marker detected
    if (ids.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
    }

    // Show annotated image
    cv::imshow(win_name, imageCopy);
}

// Detects and draws tray poses
void TrayDetector::tray_pose(cv::Mat input_img, std::string win_name)
{
    cv::Mat imageCopy = input_img.clone();
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
    cv::aruco::detectMarkers(input_img, dictionary, corners, ids);
    
    // if at least one marker detected
    if (ids.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

        // Estimate marker poses (rvecs = rotation, tvecs = translation)
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, 0.1, rgb_intrinsic, rgb_dis, rvecs, tvecs);

        for (size_t i = 0; i < ids.size(); i++)
        {
            // Draw coordinate axes on the marker
            cv::drawFrameAxes(imageCopy, rgb_intrinsic, rgb_dis, rvecs[i], tvecs[i], 0.1);

            cv::Quatd q = cv::Quatd::createFromRvec(rvecs[i]);

            // Creating tray pose message
            ariac_msgs::msg::KitTrayPose KitTray;
            geometry_msgs::msg::Pose TrayPose;
            geometry_msgs::msg::Point TrayPoint;
            geometry_msgs::msg::Quaternion TrayOrientation;

            TrayPoint.x = tvecs[i][0];
            TrayPoint.y = tvecs[i][1];
            TrayPoint.z = tvecs[i][2];
            TrayOrientation.x = q.x;
            TrayOrientation.y = q.y;
            TrayOrientation.z = q.z;
            TrayOrientation.w = q.w;

            TrayPose.position = TrayPoint;
            TrayPose.orientation = TrayOrientation;

            KitTray.id = ids[i];
            KitTray.pose = TrayPose;

            int current_id = KitTray.id; // or whatever ID you're checking for
            bool already_exists = false;

            for (const auto &tray : tray_msg_.tray_poses)
            {
                if (tray.id == current_id)
                {
                    already_exists = true;
                    break;
                }
            }

            if (!already_exists)
            {
                tray_msg_.tray_poses.push_back(KitTray);
            }

            // Log tray pose (translation + quaternion)
            // RCLCPP_INFO(this->get_logger(), "##############################################");

            // Camera frame
            // RCLCPP_INFO(this->get_logger(), "Tray %d : [%.4f %.4f %.4f] [%.4f %.4f %.4f %.4f]", ids[i], tvecs[i][0], tvecs[i][1], tvecs[i][2], q.x, q.y, q.z, q.w);

            // getting the part in world coordinate frame
            KDL::Frame kdl_camera_world;

            // kts1 Tray Pose
            if (ids[i]==1 or ids[i]==2)
                {   
                    // camera_world - constructor
                    KDL::Frame kdl_camera_world(
                        TrayDetector::ros_quaternion_to_kdl_rotation(TrayDetector::euler_to_quaternions(3.14, 3.14/2, 3.14/2)),
                        KDL::Vector(-1.3, -5.8, 1.8)
                    );

                    TrayDetector::part_world(TrayDetector::cv_to_ros_quaternions(q), tvecs[i][0], tvecs[i][1], tvecs[i][2], kdl_camera_world);

                }

            // kts2 Tray Pose
            else
                {
                    KDL::Frame kdl_camera_world(    // optical_frame_camera - constructor
                        TrayDetector::ros_quaternion_to_kdl_rotation(TrayDetector::euler_to_quaternions(3.14, 3.14/2, - 3.14/2)),
                        KDL::Vector(-1.3, 5.8, 1.8)
                    );

                    TrayDetector::part_world(TrayDetector::cv_to_ros_quaternions(q), tvecs[i][0], tvecs[i][1], tvecs[i][2], kdl_camera_world);
                }

            // RCLCPP_INFO(this->get_logger(), "##############################################");
        }
    }

    cv::imshow(win_name, imageCopy);
}

// Computes the tray's pose in the world frame given its pose in the camera frame
void TrayDetector::part_world(const geometry_msgs::msg::Quaternion &q, const double &x, const double &y, const double &z, const KDL::Frame &kdl_camera_world)
{
    // Convert poses into kdl frames
    // part_optical_frame - constructor
    KDL::Frame kdl_part_optical_frame(
        TrayDetector::ros_quaternion_to_kdl_rotation(q),
        KDL::Vector(x, y, z));

    // optical_frame_camera - constructor
    KDL::Frame kdl_optical_frame_camera(
        TrayDetector::ros_quaternion_to_kdl_rotation(TrayDetector::get_quat_msg(-0.5,0.5,-0.5,0.5)),
        KDL::Vector(0,0,0)
    );

    // Compute part - world tranformation

    KDL::Frame kdl_part_world = kdl_camera_world*kdl_optical_frame_camera*kdl_part_optical_frame;

    // Convert KDL back tp pose message
    geometry_msgs::msg::Pose pose;
    pose.position.x = kdl_part_world.p.x();
    pose.position.y = kdl_part_world.p.y();
    pose.position.z = kdl_part_world.p.z();

    pose.orientation = TrayDetector::kdl_rotation_to_ros_quaternion(kdl_part_world.M);
    auto [roll, pitch, yaw] = TrayDetector::euler_from_quaternion(pose.orientation);

    // // Print final world pose with orientation
    // RCLCPP_INFO(this->get_logger(), "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");

    // RCLCPP_INFO(this->get_logger(), "World frame : [%.4f %.4f %.4f] [%.4f %.4f %.4f]", pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw);

    // RCLCPP_INFO(this->get_logger(), "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
}

// Helper function to convert geometry_msgs::msg::Quaternion to KDL::Rotation
KDL::Rotation TrayDetector::ros_quaternion_to_kdl_rotation(const geometry_msgs::msg::Quaternion &q)
{
    return KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w);
}

// Helper function to convert euler to quaternion
geometry_msgs::msg::Quaternion TrayDetector::euler_to_quaternions(const double &roll, const double &pitch, const double &yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();
    quat_msg.w = q.w();

    return quat_msg;
}

// Helper function to convert KDL::Rotation to geometry_msgs::msg::Quaternion
geometry_msgs::msg::Quaternion TrayDetector::kdl_rotation_to_ros_quaternion(const KDL::Rotation &rot)
{
    double x, y, z, w;
    rot.GetQuaternion(x, y, z, w);
    geometry_msgs::msg::Quaternion q;
    q.x = x;
    q.y = y;
    q.z = z;
    q.w = w;
    return q;
}

// Helper function to convert cv::Quatd to geometry_msgs::msg::Quaternion
geometry_msgs::msg::Quaternion TrayDetector::cv_to_ros_quaternions(cv::Quatd q)
{
    geometry_msgs::msg::Quaternion quat;
    quat.x = q.x;
    quat.y = q.y;
    quat.z = q.z;
    quat.w = q.w;

    return quat;
}

// Helper function to convert given Quaternion to geometry_msgs::msg::Quaternion
geometry_msgs::msg::Quaternion TrayDetector::get_quat_msg(double x, double y, double z, double w)
{
    geometry_msgs::msg::Quaternion q;
    q.x = x;
    q.y = y;
    q.z = z;
    q.w = w;
    return q;
}

// Helper function to convert geometry_msgs::msg::Quaternion into Euler angles
std::tuple<double, double, double> TrayDetector::euler_from_quaternion(const geometry_msgs::msg::Quaternion &q)
{
    Eigen::Quaterniond eigen_q(q.w, q.x, q.y, q.z);
    Eigen::Vector3d euler = eigen_q.toRotationMatrix().eulerAngles(0, 1, 2);
    return std::make_tuple(euler[0], euler[1], euler[2]);
}

void TrayDetector::publish_trays_cb()
{
    detected_trays_pub_->publish(tray_msg_);
}

// Main function to initialize ROS node and start spinning
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // Initialize ROS with command-line arguments

    // Create an instance of the TrayDetector node
    auto node = std::make_shared<TrayDetector>("tray_detector");

    // Use a multithreaded executor to allow multiple callbacks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    executor.spin();

    // Shutdown ROS when done
    rclcpp::shutdown();

    return 0;
}
