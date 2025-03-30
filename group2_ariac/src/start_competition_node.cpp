#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ariac_msgs/msg/competition_state.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class CompetitionManager : public rclcpp::Node {
public:
    CompetitionManager() : Node("competition_manager") {
        competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>(
            "/ariac/competition_state", 10, std::bind(&CompetitionManager::competition_state_callback, this, _1));

        start_competition_client_ = this->create_client<std_srvs::srv::Trigger>("/ariac/start_competition");

        RCLCPP_INFO(this->get_logger(), "Competition Manager Node Initialized");
    }

private:
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_competition_client_;

    void competition_state_callback(const ariac_msgs::msg::CompetitionState::SharedPtr msg) {
        if (msg->competition_state == ariac_msgs::msg::CompetitionState::READY) {
            RCLCPP_INFO(this->get_logger(), "Competition is READY. Starting competition...");

            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            while (!start_competition_client_->wait_for_service(1s)) {
                RCLCPP_WARN(this->get_logger(), "Waiting for /ariac/start_competition service...");
            }

            auto result = start_competition_client_->async_send_request(request);
            result.wait();
            if (result.get()->success) {
                RCLCPP_INFO(this->get_logger(), "Competition started successfully.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to start competition: %s", result.get()->message.c_str());
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CompetitionManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
