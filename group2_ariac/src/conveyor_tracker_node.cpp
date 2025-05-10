#include "conveyor_tracker_node.hpp"

ConveyorTracker::ConveyorTracker()
    : Node("conveyor_tracker_node")
{
    RCLCPP_INFO(this->get_logger(), "Conveyor Tracker Node Started");

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/ariac/sensors/ceiling_conveyor_camera/rgb_image", 10,
        std::bind(&ConveyorTracker::imageCallback, this, std::placeholders::_1));
}

ConveyorTracker::~ConveyorTracker()
{
    cv::destroyAllWindows();
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

    int image_center_x = image.cols / 2;

    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    cv::Scalar blue_lower(100, 100, 100), blue_upper(130, 255, 255);
    cv::Scalar purple_lower(130, 50, 50), purple_upper(160, 255, 255);

    cv::Mat blue_mask, purple_mask;
    cv::inRange(hsv, blue_lower, blue_upper, blue_mask);
    cv::inRange(hsv, purple_lower, purple_upper, purple_mask);

    std::vector<std::vector<cv::Point>> contours;

    // --- BLUE Detection ---
    cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto &c : contours) {
        if (cv::contourArea(c) > 2000) {
            cv::RotatedRect rbox = cv::minAreaRect(c);
            cv::Point2f vertices[4];
            rbox.points(vertices);
            for (int i = 0; i < 4; ++i) {
                cv::line(image, vertices[i], vertices[(i + 1) % 4], cv::Scalar(255, 0, 0), 2);
            }

            cv::Point2f centroid = rbox.center;

            if (std::abs(centroid.x - image_center_x) < 5) {
                std::string key = "blue_" + std::to_string(static_cast<int>(centroid.x)) + "_" + std::to_string(static_cast<int>(centroid.y));
                if (seen_objects_.find(key) == seen_objects_.end()) {
                    seen_objects_.insert(key);

                    RCLCPP_INFO(this->get_logger(), "BLUE angle: %.2f", rbox.angle);

                    double cam_x = -0.6, cam_y = 3.63, cam_z = 1.35, obj_z = 0.875;
                    double belt_speed_ = 0.345;
                    double y = cam_y - (cam_z - obj_z);
                    RCLCPP_INFO(this->get_logger(), "BLUE BATTERY at [%.3f, %.3f, %.3f]", cam_x, y, obj_z);
                    for (int sec = 1; sec <= 2; ++sec) {
                        double pred_y = y - belt_speed_ * sec;
                        RCLCPP_INFO(this->get_logger(),
                            "Prediction [%ds]: at [%.3f, %.3f, %.3f]",
                            sec, cam_x, pred_y, obj_z);
                    }
                }
            }
        }
    }

    // --- PURPLE Detection ---
    cv::findContours(purple_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto &c : contours) {
        if (cv::contourArea(c) > 2000) {
            cv::RotatedRect rbox = cv::minAreaRect(c);
            cv::Point2f vertices[4];
            rbox.points(vertices);
            for (int i = 0; i < 4; ++i) {
                cv::line(image, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
            }

            cv::Point2f centroid = rbox.center;
            if (std::abs(centroid.x - image_center_x) < 5) {
                std::string key = "purple_" + std::to_string(static_cast<int>(centroid.x)) + "_" + std::to_string(static_cast<int>(centroid.y));
                if (seen_objects_.find(key) == seen_objects_.end()) {
                    seen_objects_.insert(key);

                    RCLCPP_INFO(this->get_logger(), "PURPLE angle: %.2f", rbox.angle);

                    double cam_x = -0.6, cam_y = 3.63, cam_z = 1.35, obj_z = 0.875;
                    double belt_speed_ = 0.345;
                    double y = cam_y - (cam_z - obj_z);
                    RCLCPP_INFO(this->get_logger(), "PURPLE PUMP at [%.3f, %.3f, %.3f]", cam_x, y, obj_z);
                    for (int sec = 1; sec <= 2; ++sec) {
                        double pred_y = y - belt_speed_ * sec;
                        RCLCPP_INFO(this->get_logger(),
                            "Prediction [%ds]: at [%.3f, %.3f, %.3f]",
                            sec, cam_x, pred_y, obj_z);
                    }
                }
            }
        }
    }

    cv::imshow("Conveyor Tracker View", image);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConveyorTracker>());
    rclcpp::shutdown();
    return 0;
}
