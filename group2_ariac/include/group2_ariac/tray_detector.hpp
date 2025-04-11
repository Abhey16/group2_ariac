#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>


class TrayDectector : public rclcpp::Node
{
    public:

        TrayDectector(std::string node_name) : Node(node_name)
        {
            kt_right_sub_ = this->create_subscription<sensor_msgs::msg::Image>
                                    ("/ariac/sensors/rgb_camera_kts1/rgb_image",
                                        10,
                                        std::bind(&TrayDectector::kt_right_cb,this,std::placeholders::_1));

            kt_left_sub_ = this->create_subscription<sensor_msgs::msg::Image>
                                    ("/ariac/sensors/rgb_camera_kts2/rgb_image",
                                        10,
                                        std::bind(&TrayDectector::kt_left_cb,this,std::placeholders::_1));
            
            RCLCPP_INFO(this->get_logger(),"tray detector class created");
        }

        void kt_right_cb(sensor_msgs::msg::Image::ConstSharedPtr img);
        void kt_left_cb(sensor_msgs::msg::Image::ConstSharedPtr img);
    
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr kt_right_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr kt_left_sub_;

        cv::Mat kt_right_rgb {};
        cv::Mat kt_left_rgb {};
        
};