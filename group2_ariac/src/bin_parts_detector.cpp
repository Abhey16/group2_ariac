#include "bin_parts_detector.hpp"

// Left Camera Callback
void BinPartsDetector::bin_left_cb(sensor_msgs::msg::Image::ConstSharedPtr img)
{
    bin_left_rgb_ = cv_bridge::toCvShare(img, "bgr8")->image;
    left_image_ready_ = true;
}

// Right Camera Callback
void BinPartsDetector::bin_right_cb(sensor_msgs::msg::Image::ConstSharedPtr img)
{
    bin_right_rgb_ = cv_bridge::toCvShare(img, "bgr8")->image;
    right_image_ready_ = true;
}

// Callback that initiates part detection
void BinPartsDetector::detect_parts_cb()
{
    // Don't do anything if we haven't received an image
    if (!(left_image_ready_ && right_image_ready_))
    {
        // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        //                      "Waiting for both images to be received...");
        return;
    }

    // Reset flags immediately to wait for next valid pair
    left_image_ready_ = false;
    right_image_ready_ = false;

    // Clear the previous detections
    detected_parts_.clear();

    // Loop through every bin
    for (int i = 1; i <= 8; i++)
    {
        // Find and add detected parts to the list
        get_bin_parts(i);
    }

    group2_msgs::msg::PartList msg;

    for (const auto &part : detected_parts_)
    {
        group2_msgs::msg::Part part_msg;
        part_msg.color = part.color;
        part_msg.type = part.type;
        part_msg.bin_number = part.bin_number;
        part_msg.location = "bin" + std::to_string(part.bin_number);
        part_msg.position = part.position;
        part_msg.orientation = part.orientation;

        msg.parts.push_back(part_msg);
    }

    detected_parts_pub_->publish(msg);

    // print_detected_parts();
}

// Find all parts for the given bin number
void BinPartsDetector::get_bin_parts(const int &bin_number)
{
    if (!bin_left_rgb_.empty() && !bin_right_rgb_.empty())
    {
        // Get the raw image
        cv::Mat cv_img = (bin_number > 4) ? bin_left_rgb_ : bin_right_rgb_;
        int imgH = cv_img.rows;
        int imgW = cv_img.cols;

        // Define ROI based on bin number
        cv::Mat roi_img;
        cv::Point2f roi_offset;

        if (bin_number == 1 || bin_number == 6)
        {
            roi_img = cv_img(cv::Range(imgH / 2, imgH), cv::Range((imgW / 2) + 20, imgW - 100));
            roi_offset = cv::Point2f((imgW / 2.0f) + 20.0f, imgH / 2.0f);
        }
        else if (bin_number == 2 || bin_number == 5)
        {
            roi_img = cv_img(cv::Range(imgH / 2, imgH), cv::Range(100, (imgW / 2) - 20));
            roi_offset = cv::Point2f(100.0f, imgH / 2.0f);
        }
        else if (bin_number == 3 || bin_number == 8)
        {
            roi_img = cv_img(cv::Range(0, imgH / 2), cv::Range(100, (imgW / 2) - 20));
            roi_offset = cv::Point2f(100.0f, 0.0f);
        }
        else if (bin_number == 4 || bin_number == 7)
        {
            roi_img = cv_img(cv::Range(0, imgH / 2), cv::Range((imgW / 2) + 20, imgW - 100));
            roi_offset = cv::Point2f((imgW / 2.0f) + 20.0f, 0.0f);
        }

        // Find all parts in the defined roi
        find_parts(roi_img);

        // Define which camera frame we must use to get world pose
        std::string camera_frame = (bin_number > 4) ? "rgb_left_bins_optical_frame" : "rgb_right_bins_optical_frame";
        double depth = 1.02; // Estimate

        // Loop through every detection and find their world poses
        for (const auto &[color, type_map] : part_roi_coordinates_)
        {
            for (const auto &[type, centers] : type_map)
            {
                for (const auto &pixel : centers)
                {
                    // Convert roi part coordinate to full view coordinate
                    cv::Point2f full_pixel = cv::Point2f(pixel) + roi_offset;
                    // Convert camera pixel coordinate to a pose in the camera optical frame
                    geometry_msgs::msg::PointStamped cam_pose = pixel_to_optical_frame(full_pixel, camera_frame, depth);
                    // World pose definition
                    geometry_msgs::msg::PointStamped world_pose;

                    try
                    {
                        // Pose transformation from optical_camera_frame and world_frame
                        tf_buffer_->transform(cam_pose, world_pose, "world", tf2::durationFromSec(0.1));
                    }
                    catch (tf2::TransformException &ex)
                    {
                        // RCLCPP_WARN(this->get_logger(), "TF2 Transform failed: %s", ex.what());
                        continue; // Skip this point and move on
                    }

                    // Create and fill part object with detection information
                    Part part;
                    part.color = color;
                    part.type = type;
                    part.bin_number = bin_number;
                    part.position = world_pose.point;
                    part.orientation.w = 1.0;

                    detected_parts_.push_back(part); // Add the part to the list
                }
            }
        }
    }
}

// Find parts in an image based on color and templates
void BinPartsDetector::find_parts(const cv::Mat &img)
{
    // Converting from BGR to HSV
    cv::Mat img_hsv;
    cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

    // HSV Mask Color Bounds
    std::map<std::string, std::pair<cv::Scalar, cv::Scalar>> hsv_colors{
        {"red", {cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255)}},
        {"green", {cv::Scalar(40, 70, 70), cv::Scalar(85, 255, 255)}},
        {"blue", {cv::Scalar(100, 150, 0), cv::Scalar(130, 255, 255)}},
        {"orange", {cv::Scalar(10, 150, 150), cv::Scalar(25, 255, 255)}},
        {"purple", {cv::Scalar(130, 50, 50), cv::Scalar(160, 255, 255)}}};

    // Applying all masks and running template matching on each result
    for (const auto &[color, bounds] : hsv_colors)
    {
        // Apply mask
        cv::Mat img_mask;
        cv::inRange(img_hsv, bounds.first, bounds.second, img_mask);

        // Run template matching for each part type
        for (const auto &type : types_)
        {
            // Apply template to color mask result
            match_template(img_mask, color, type);
        }
    }
}

// Match a detected part to a part type using a template
void BinPartsDetector::match_template(const cv::Mat &img_mask,
                                      const std::string &color,
                                      const std::string &type)
{
    cv::Mat template_img;
    if (type == "pump")
        template_img = pump_template_;
    else if (type == "battery")
        template_img = battery_template_;
    else if (type == "sensor")
        template_img = sensor_template_;
    else if (type == "regulator")
        template_img = regulator_template_;
    else
        return;

    int tH = template_img.rows;
    int tW = template_img.cols;

    // Template matching
    cv::Mat match_field;
    cv::matchTemplate(img_mask, template_img, match_field, cv::TM_CCOEFF_NORMED);

    std::vector<cv::Rect> raw_matches;
    for (int y = 0; y < match_field.rows; ++y)
    {
        for (int x = 0; x < match_field.cols; ++x)
        {
            float confidence = match_field.at<float>(y, x);
            if (confidence >= 0.80f)
            {
                raw_matches.emplace_back(x, y, tW, tH);
            }
        }
    }

    // Refining matches by applying non max suppression
    std::vector<cv::Rect> refined_matches = non_max_suppression(raw_matches);

    // Computing centers (turning matches into points)
    std::vector<cv::Point> centered_matches;
    for (const auto &rect : refined_matches)
    {
        centered_matches.emplace_back(rect.x + rect.width / 2, rect.y + rect.height / 2);
    }

    // Store results
    part_roi_coordinates_[color][type] = centered_matches;
}

// Non max suppresion method used to filter multiple detections
// Over-simplified aproach based on python's imutil module
std::vector<cv::Rect> BinPartsDetector::non_max_suppression(const std::vector<cv::Rect> &boxes, float overlap_threshold)
{
    std::vector<cv::Rect> result;
    if (boxes.empty())
        return result;

    std::vector<cv::Rect> sorted_boxes = boxes;
    std::sort(sorted_boxes.begin(), sorted_boxes.end(), [](const cv::Rect &a, const cv::Rect &b)
              {
                  return a.area() > b.area(); // Sort by area (can also sort by response later)
              });
    while (!sorted_boxes.empty())
    {
        cv::Rect current = sorted_boxes[0];
        result.push_back(current);
        std::vector<cv::Rect> remaining;

        for (size_t i = 1; i < sorted_boxes.size(); ++i)
        {
            float intersection_area = (current & sorted_boxes[i]).area();
            float union_area = current.area() + sorted_boxes[i].area() - intersection_area;
            float iou = intersection_area / union_area;

            if (iou < overlap_threshold)
            {
                remaining.push_back(sorted_boxes[i]);
            }
        }
        sorted_boxes = remaining;
    }
    return result;
}

// Broadcasting static transforms from camre optical frames to world
void BinPartsDetector::broadcast_camera_transforms()
{
    // Adding optical frames
    std::vector<std::pair<std::string, std::string>> optical_links = {
        {"rgb_left_bins_frame", "rgb_left_bins_optical_frame"},
        {"rgb_right_bins_frame", "rgb_right_bins_optical_frame"}};

    for (const auto &[parent_frame, optical_frame] : optical_links)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = parent_frame;
        t.child_frame_id = optical_frame;

        tf2::Quaternion q;
        q.setRPY(-M_PI / 2, 0, -M_PI); // Frame rotation based on camera placement

        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;

        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        static_broadcaster_->sendTransform(t); // Broadcasting the transform
    }
}

// Convert image coordinates into pose from the optical_frame
geometry_msgs::msg::PointStamped BinPartsDetector::pixel_to_optical_frame(const cv::Point2f &pixel,
                                                                          const std::string &optical_frame,
                                                                          double depth)
{
    geometry_msgs::msg::PointStamped cam_point;

    // Fill header
    cam_point.header.frame_id = optical_frame;
    cam_point.header.stamp = this->get_clock()->now();

    // Getting camera intrinsic parameters
    double fx = rgb_intrinsic_.at<double>(0, 0);
    double fy = rgb_intrinsic_.at<double>(1, 1);
    double cx = rgb_intrinsic_.at<double>(0, 2);
    double cy = rgb_intrinsic_.at<double>(1, 2);

    // Convert pixel to 3D point in camera optical frame
    cam_point.point.x = (pixel.x - cx) * depth / fx;
    cam_point.point.y = (pixel.y - cy) * depth / fy;
    cam_point.point.z = depth + 0.06;

    return cam_point;
}

// Print out detected parts for debugging
void BinPartsDetector::print_detected_parts()
{
    if (!detected_parts_.empty())
        RCLCPP_INFO(this->get_logger(), "- Parts:");
    for (const auto &part : detected_parts_)
    {
        RCLCPP_INFO(this->get_logger(), "  - %s %s:", part.color.c_str(), part.type.c_str());
        RCLCPP_INFO(this->get_logger(), "    - Location: bin%d", part.bin_number);
        RCLCPP_INFO(this->get_logger(), "    - [%.3f, %.3f, %.3f] [%.1f, %.1f, %.1f, %.1f]",
                    part.position.x, part.position.y, part.position.z,
                    part.orientation.x, part.orientation.y, part.orientation.z, part.orientation.w);
    }
    if (!detected_parts_.empty())
        RCLCPP_INFO(this->get_logger(), " ");
}

// Loads templates for part matching
bool BinPartsDetector::load_part_templates()
{
    std::string package_path = ament_index_cpp::get_package_share_directory("group2_ariac");
    std::string template_path = package_path + "/resources/partTemplateMasks/";

    sensor_template_ = cv::imread(template_path + "sensor.png", cv::IMREAD_GRAYSCALE);
    regulator_template_ = cv::imread(template_path + "regulator.png", cv::IMREAD_GRAYSCALE);
    battery_template_ = cv::imread(template_path + "battery.png", cv::IMREAD_GRAYSCALE);
    pump_template_ = cv::imread(template_path + "pump.png", cv::IMREAD_GRAYSCALE);

    // One or more part templates not found
    if (sensor_template_.empty() ||
        regulator_template_.empty() ||
        battery_template_.empty() ||
        pump_template_.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "One or more part templates not found.");
        return false;
    }
    // RCLCPP_INFO(this->get_logger(), "Part templates loaded");
    return true;
}

// Main function to initialize ROS node and start spinning
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // Initialize ROS with command-line arguments

    // Create a BinPartsDetector node instance
    auto node = std::make_shared<BinPartsDetector>("bin_parts_detector");

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);

    executor.spin(); // This will start the execution

    // Shutdown ROS when done
    rclcpp::shutdown();

    return 0;
}