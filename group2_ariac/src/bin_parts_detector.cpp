#include "bin_parts_detector.hpp"

void BinPartsDetector::bin_left_cb(sensor_msgs::msg::Image::ConstSharedPtr img)
{

    bin_left_rgb_ = cv_bridge::toCvShare(img, "bgr8")->image;

    for (int i = 5; i <= 8; i++)
    {
        get_bin_parts(i);
        if (debug_bins_)
            print_slot_assignments(i);
    }

    if (enable_left_camera_)
    {
        cv::imshow("Display window - left", bin_left_rgb_);
        cv::waitKey(1); // Wait for a keystroke in the window
    }
}

void BinPartsDetector::bin_right_cb(sensor_msgs::msg::Image::ConstSharedPtr img)
{
    bin_right_rgb_ = cv_bridge::toCvShare(img, "bgr8")->image;

    for (int i = 1; i <= 4; i++)
    {
        get_bin_parts(i);
        if (debug_bins_)
            print_slot_assignments(i);
    }

    if (enable_right_camera_)
    {
        cv::imshow("Display window - right", bin_right_rgb_);
        cv::waitKey(1); // Wait for a keystroke in the window
    }
}

void BinPartsDetector::get_bin_parts(const int &bin_number)
{
    // Make sure we have valid images
    if (!bin_left_rgb_.empty() && !bin_right_rgb_.empty())
    {
        cv::Mat cv_img;
        if (bin_number > 4)
        {
            // Use left camera for bins 5-8
            cv_img = bin_left_rgb_;
        }
        else
        {
            // Use right camera for bins 1-4
            cv_img = bin_right_rgb_;
        }

        int imgH = cv_img.rows;
        int imgW = cv_img.cols;

        // Define ROI based on bin number
        cv::Mat roi_img;
        if (bin_number == 1 || bin_number == 6)
        {
            // Bottom left
            roi_img = cv_img(cv::Range(imgH / 2, imgH), cv::Range((imgW / 2) + 20, imgW - 100));
        }
        else if (bin_number == 2 || bin_number == 5)
        {
            // Bottom right
            roi_img = cv_img(cv::Range(imgH / 2, imgH), cv::Range(100, (imgW / 2) - 20));
        }
        else if (bin_number == 3 || bin_number == 8)
        {
            // Top left
            roi_img = cv_img(cv::Range(0, imgH / 2), cv::Range(100, (imgW / 2) - 20));
        }
        else if (bin_number == 4 || bin_number == 7)
        {
            // Top right
            roi_img = cv_img(cv::Range(0, imgH / 2), cv::Range((imgW / 2) + 20, imgW - 100));
        }

        find_parts(roi_img);

        // Choose the frame name depending on the bin
        std::string camera_frame = (bin_number > 4) ? "rgb_left_bins_frame" : "rgb_right_bins_frame";

        // Assume known constant depth (e.g. measured tray height)
        double depth = 1.0; // ← Tune this!

        for (const auto &[color, type_map] : centered_part_poses_)
        {
            for (const auto &[type, centers] : type_map)
            {
                for (const auto &pixel : centers)
                {
                    geometry_msgs::msg::Point world_point = pixelToWorldPoint(pixel, camera_frame, depth);

                    PartInstance part;
                    part.color = color;
                    part.type = type;
                    part.bin_number = bin_number;
                    part.position = world_point;
                    part.orientation.w = 1.0;

                    bool exists = std::any_of(detected_parts_.begin(), detected_parts_.end(), [&](const PartInstance& p) {
                        return p.color == part.color &&
                               p.type == part.type &&
                               p.bin_number == part.bin_number &&
                               std::fabs(p.position.x - part.position.x) < 1e-3 &&
                               std::fabs(p.position.y - part.position.y) < 1e-3 &&
                               std::fabs(p.position.z - part.position.z) < 1e-3;
                    });

                    if (!exists)
                    {
                        detected_parts_.push_back(part);
                    }
                }
            }
        }

        print_all_detected_parts();

        // Display the cropped ROI for debugging
        if (roi_debug_)
        {
            cv::imshow("Bin ROI", roi_img);
            cv::waitKey(1);
        }
    }
}

void BinPartsDetector::find_parts(const cv::Mat &img)
{
    // Convert BGR to HSV
    cv::Mat imgHSV;
    cv::cvtColor(img, imgHSV, cv::COLOR_BGR2HSV);

    // Example color bounds (you'll need to define your own or use a map)
    std::map<std::string, std::pair<cv::Scalar, cv::Scalar>> color_bounds{
        // Hue is from 0-180 in OpenCV, Saturation & Value from 0-255
        // Red has two ranges due to hue wrapping around 0
        {"red", {cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255)}},
        {"green", {cv::Scalar(40, 70, 70), cv::Scalar(85, 255, 255)}},
        {"blue", {cv::Scalar(100, 150, 0), cv::Scalar(130, 255, 255)}},
        {"orange", {cv::Scalar(10, 150, 150), cv::Scalar(25, 255, 255)}},
        {"purple", {cv::Scalar(130, 50, 50), cv::Scalar(160, 255, 255)}}};

    for (const auto &[color, bounds] : color_bounds)
    {
        // Apply mask
        cv::Mat imgMask;
        cv::inRange(imgHSV, bounds.first, bounds.second, imgMask);

        // For each part type, run template matching
        std::vector<std::string> part_types = {"battery", "pump", "sensor", "regulator"};
        for (const auto &type : part_types)
        {
            match_template(imgMask, color, type);
        }
        // Optional: show the mask for debugging
        // std::string window_name = "Mask - " + color;
        // cv::imshow(window_name, imgMask);
        // cv::waitKey(1);

        // TODO: Find contours, bounding boxes, centroids, etc.
    }
}

void BinPartsDetector::match_template(const cv::Mat &imgMask, const std::string &color, const std::string &type)
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
    cv::Mat matchField;
    cv::matchTemplate(imgMask, template_img, matchField, cv::TM_CCOEFF_NORMED);

    std::vector<cv::Rect> raw_matches;
    for (int y = 0; y < matchField.rows; ++y)
    {
        for (int x = 0; x < matchField.cols; ++x)
        {
            float confidence = matchField.at<float>(y, x);
            if (confidence >= 0.80f)
            {
                raw_matches.emplace_back(x, y, tW, tH);
            }
        }
    }

    // Apply NMS
    std::vector<cv::Rect> refined_matches = non_max_suppression(raw_matches);

    // Compute centers
    std::vector<cv::Point> centered_matches;
    for (const auto &rect : refined_matches)
    {
        centered_matches.emplace_back(rect.x + rect.width / 2, rect.y + rect.height / 2);
    }

    // Store results
    part_poses_[color][type] = refined_matches;
    centered_part_poses_[color][type] = centered_matches;

    // Only display if we found any matches
    if (!centered_matches.empty() && (enable_left_camera_ || enable_right_camera_))
    {
        // Clone the mask to visualize results
        cv::Mat display;
        cv::cvtColor(imgMask, display, cv::COLOR_GRAY2BGR); // Convert to color for drawing

        // Draw matches
        for (const auto &center : centered_matches)
        {
            cv::circle(display, center, 8, cv::Scalar(0, 255, 255), 2); // yellow-ish circle
            cv::putText(display, type, center + cv::Point(10, 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        }

        // Show window titled by color + type
        std::string window_name = "Matches - " + color + " " + type;
        cv::imshow(window_name, display);
        cv::waitKey(1);
    }
}

std::vector<cv::Rect> BinPartsDetector::non_max_suppression(const std::vector<cv::Rect> &boxes, float overlapThreshold)
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
            float intersectionArea = (current & sorted_boxes[i]).area();
            float unionArea = current.area() + sorted_boxes[i].area() - intersectionArea;
            float iou = intersectionArea / unionArea;

            if (iou < overlapThreshold)
            {
                remaining.push_back(sorted_boxes[i]);
            }
        }

        sorted_boxes = remaining;
    }

    return result;
}

std::map<int, BinPartsDetector::PartMsg> BinPartsDetector::output_by_slot()
{
    std::map<int, PartMsg> bin;

    // Initialize bin slots 1–9 with empty parts
    for (int i = 1; i <= 9; ++i)
    {
        bin[i] = PartMsg{"", ""};
    }

    for (const auto &[color, type_map] : centered_part_poses_)
    {
        for (const auto &[type, centers] : type_map)
        {
            for (const auto &pt : centers)
            {
                int row = 0;
                if (pt.y <= 88)
                    row = 1;
                else if (pt.y >= 151)
                    row = 3;
                else
                    row = 2;

                int col = 0;
                if (pt.x <= 68)
                    col = 1;
                else if (pt.x >= 131)
                    col = 3;
                else
                    col = 2;

                auto slot_it = slot_mapping_.find({row, col});
                if (slot_it != slot_mapping_.end())
                {
                    int slot = slot_it->second;
                    bin[slot] = PartMsg{color, type};
                }
            }
        }
    }

    return bin;
}

void BinPartsDetector::print_slot_assignments(int bin_number)
{
    auto bin = output_by_slot();

    RCLCPP_INFO(this->get_logger(), "=== Slot Assignments for Bin %d ===", bin_number);

    for (int slot = 1; slot <= 9; ++slot)
    {
        const auto &part = bin[slot];
        if (!part.color.empty() && !part.type.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Slot %d: %s %s", slot, part.color.c_str(), part.type.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Slot %d: Empty", slot);
        }
    }
    RCLCPP_INFO(this->get_logger(), "===================================");
}

geometry_msgs::msg::Point BinPartsDetector::pixelToWorldPoint(const cv::Point2f &pixel,
                                                              const std::string &camera_frame,
                                                              double depth)
{
    geometry_msgs::msg::PointStamped cam_point, world_point;

    cam_point.header.frame_id = camera_frame;
    cam_point.header.stamp = this->get_clock()->now();

    // Project pixel to 3D in camera frame
    cv::Mat uv_hom = (cv::Mat_<double>(3, 1) << pixel.x, pixel.y, 1.0);
    cv::Mat xyz_cam = rgb_intrinsic_.inv() * uv_hom * depth;

    cam_point.point.x = xyz_cam.at<double>(0);
    cam_point.point.y = xyz_cam.at<double>(1);
    cam_point.point.z = xyz_cam.at<double>(2);

    try
    {
        tf_buffer_->transform(cam_point, world_point, "world", tf2::durationFromSec(0.1));
        return world_point.point;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
        return geometry_msgs::msg::Point(); // Default zero point
    }
}

void BinPartsDetector::print_all_detected_parts()
{
    RCLCPP_INFO(this->get_logger(), "- Parts:");
    for (const auto& part : detected_parts_)
    {
        RCLCPP_INFO(this->get_logger(), "  - %s %s:", part.color.c_str(), part.type.c_str());
        RCLCPP_INFO(this->get_logger(), "    - Location: bin%d", part.bin_number);
        RCLCPP_INFO(this->get_logger(), "    - [%.3f, %.3f, %.3f] [%.1f, %.1f, %.1f, %.1f]\n",
                    part.position.x, part.position.y, part.position.z,
                    part.orientation.x, part.orientation.y, part.orientation.z, part.orientation.w);
    }
}

bool BinPartsDetector::load_part_templates()
{
    std::string package_path = ament_index_cpp::get_package_share_directory("group2_ariac");
    std::string template_path = package_path + "/resources/partTemplateMasks/";

    sensor_template_ = cv::imread(template_path + "sensor.png", cv::IMREAD_GRAYSCALE);
    regulator_template_ = cv::imread(template_path + "regulator.png", cv::IMREAD_GRAYSCALE);
    battery_template_ = cv::imread(template_path + "battery.png", cv::IMREAD_GRAYSCALE);
    pump_template_ = cv::imread(template_path + "pump.png", cv::IMREAD_GRAYSCALE);

    if (sensor_template_.empty() ||
        regulator_template_.empty() ||
        battery_template_.empty() ||
        pump_template_.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "One or more part templates could not be loaded.");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Part templates successfully loaded.");
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
