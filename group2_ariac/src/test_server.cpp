#include "rclcpp/rclcpp.hpp"
#include "group2_msgs/srv/pose.hpp"
#include <chrono>

using namespace std::placeholders;
using namespace std::chrono_literals;

class TestServer : public rclcpp::Node
{
public:
    TestServer() : Node("test_server")
    {
        server_ = this->create_service<group2_msgs::srv::Pose>(
            "move_it_pose",
            std::bind(&TestServer::callback_test_server, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Test Service has been started.");
    }

private:
    void callback_test_server(const group2_msgs::srv::Pose::Request::SharedPtr request,
                            const group2_msgs::srv::Pose::Response::SharedPtr response)
    {   

        RCLCPP_INFO(this->get_logger(),"recieve client request");
        RCLCPP_INFO(this->get_logger(),"type %s",request->type.c_str());
        
        if (request->type == "tray")
            RCLCPP_INFO(this->get_logger(),"agv :%d",request->agv_number);
        else
            RCLCPP_INFO(this->get_logger(),"type :%s",request->quadrant.c_str());

        rclcpp::sleep_for(5s);
        response->status = true;

        RCLCPP_INFO(this->get_logger(),"status %d",response->status);

    }

    rclcpp::Service<group2_msgs::srv::Pose>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
