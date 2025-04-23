#include "conveyor_tracker_node.hpp"

ConveyorTracker::ConveyorTracker()
    : Node("conveyor_tracker_node")
{
    RCLCPP_INFO(this->get_logger(), "Conveyor Tracker Node Started");

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/ariac/sensors/ceiling_conveyor_camera/rgb_image", 10,
        std::bind(&ConveyorTracker::imageCallback, this, std::placeholders::_1));

    // Initialize timer to run part detection
    publish_parts_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ConveyorTracker::publish_parts_cb, this));

    // Initialize publisher for detected parts
    detected_parts_pub_ = this->create_publisher<group2_msgs::msg::PartList>("detected_conveyor_parts", 10);        
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
            cv::Moments mu = cv::moments(c);

            if (mu.m00 == 0) continue;

            cv::Point2f centroid(mu.m10 / mu.m00, mu.m01 / mu.m00);
            if (std::abs(centroid.x - image_center_x) < 5) {

                double cam_x = -0.6;
                double cam_y = 3.63;
                double cam_z = 1.35;
                double obj_z = 0.875;
                double belt_speed_ = 0.345;

                double y = cam_y - (cam_z - obj_z);

                group2_msgs::msg::Part part_msg;
                part_msg.color = "blue";
                part_msg.type = "battery";
                part_msg.location = "conveyor";
                part_msg.position.x = cam_x;
                part_msg.position.y = y;
                part_msg.position.z = obj_z;
                part_msg.initial_pos.x = cam_x;
                part_msg.initial_pos.y = y;
                part_msg.initial_pos.z = obj_z;

                // Prediction for 1s
                double pred_y_1 = y - belt_speed_ * 1;
                part_msg.predicted_pos_1.x = cam_x;
                part_msg.predicted_pos_1.y = pred_y_1;
                part_msg.predicted_pos_1.z = obj_z;

                // Prediction for 2s
                double pred_y_2 = y - belt_speed_ * 2;
                part_msg.predicted_pos_2.x = cam_x;
                part_msg.predicted_pos_2.y = pred_y_2;
                part_msg.predicted_pos_2.z = obj_z;                

                detections_msg_.parts.push_back(part_msg);

                // RCLCPP_INFO(this->get_logger(), "BLUE BATTERY at [%.3f, %.3f, %.3f]", cam_x, y, obj_z);
                // predic 1s, 2s later location
                // for (int sec = 1; sec <= 2; ++sec) {
                //     double pred_y = y - belt_speed_ * sec;
                //     RCLCPP_INFO(this->get_logger(),
                //     "Prediction [%ds]: at [%.3f, %.3f, %.3f]",
                //     sec, cam_x, pred_y, obj_z);
                // }
            }
        }
    }

    cv::findContours(purple_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& c : contours) {
        if (cv::contourArea(c) > 2000) {
            cv::Moments mu = cv::moments(c);

            if (mu.m00 == 0) continue;

            cv::Point2f centroid(mu.m10 / mu.m00, mu.m01 / mu.m00);
            if (std::abs(centroid.x - image_center_x) < 5) {

                double cam_x = -0.6;
                double cam_y = 3.63;
                double cam_z = 1.35;
                double obj_z = 0.875;
                double belt_speed_ = 0.345;

                double y = cam_y - (cam_z - obj_z);

                group2_msgs::msg::Part part_msg;
                part_msg.color = "purple";
                part_msg.type = "pump";
                part_msg.location = "conveyor";
                part_msg.position.x = cam_x;
                part_msg.position.y = y;
                part_msg.position.z = obj_z;
        
                // Prediction for 1s
                double pred_y_1 = y - belt_speed_ * 1;
                part_msg.predicted_pos_1.x = cam_x;
                part_msg.predicted_pos_1.y = pred_y_1;
                part_msg.predicted_pos_1.z = obj_z;

                // Prediction for 2s
                double pred_y_2 = y - belt_speed_ * 2;
                part_msg.predicted_pos_2.x = cam_x;
                part_msg.predicted_pos_2.y = pred_y_2;
                part_msg.predicted_pos_2.z = obj_z;                

                detections_msg_.parts.push_back(part_msg);

                // RCLCPP_INFO(this->get_logger(), "PURPLE PUMP at [%.3f, %.3f, %.3f]", cam_x, y, obj_z);
                // // predic 1s, 2s later location
                // for (int sec = 1; sec <= 2; ++sec) {
                //     double pred_y = y - belt_speed_ * sec;
                //     RCLCPP_INFO(this->get_logger(),
                //     "Prediction [%ds]: at [%.3f, %.3f, %.3f]",
                //     sec, cam_x, pred_y, obj_z);
                // }
            }
        }
    }
    if (first_detection_) 
    {
        update_timer_ = 1.0;
    }
}

void ConveyorTracker::publish_parts_cb()
{
    update_timer_ += 0.1;

    if (!first_detection_ && update_timer_ >= 1.0) 
    {
        detected_parts_pub_->publish(detections_msg_);
        update_timer_ = 0;

        double belt_speed_ = 0.345;

        for (auto &part : detections_msg_.parts)
        {
            part.position = part.predicted_pos_1;

            double x = part.position.x;
            double y = part.position.y;
            double z = part.position.z;

            part.predicted_pos_1.x = x;
            part.predicted_pos_1.y = y - belt_speed_ * 1;
            part.predicted_pos_1.z = z;

            part.predicted_pos_2.x = x;
            part.predicted_pos_2.y = y - belt_speed_ * 2;
            part.predicted_pos_2.z = z;
        }
        detections_msg_.parts.erase(
            std::remove_if(detections_msg_.parts.begin(),
                           detections_msg_.parts.end(),
                           [](const group2_msgs::msg::Part &part) {
                                return part.position.y >= 4.72;
                            }),
            detections_msg_.parts.end()
        );
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
