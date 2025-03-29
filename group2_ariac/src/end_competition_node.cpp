#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ariac_msgs/msg/competition_state.hpp"

using std::placeholders::_1;

class EndCompetitionNode : public rclcpp::Node {
public:
    EndCompetitionNode() : Node("end_competition_node") {
        // end competition client
        end_competition_client_ = this->create_client<std_srvs::srv::Trigger>("/ariac/end_competition");

        // subsribe competition state
        state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>(
            "/ariac/competition_state", 10,
            std::bind(&EndCompetitionNode::state_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "End Competition Node Initialized");
    }

private:
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr state_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr end_competition_client_;
    bool ended_ = false;

    void state_callback(const ariac_msgs::msg::CompetitionState::SharedPtr msg) {
        if (ended_) return;

        if (msg->competition_state == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE) {
            RCLCPP_INFO(this->get_logger(), "Competition state is ORDER_ANNOUNCEMENTS_DONE. Ending competition...");

            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

            while (!end_competition_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(), "Waiting for /ariac/end_competition service...");
            }

            auto future = end_competition_client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
                rclcpp::FutureReturnCode::SUCCESS) {
                if (future.get()->success) {
                    RCLCPP_INFO(this->get_logger(), "Competition ended successfully.");
                    ended_ = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to end competition: %s", future.get()->message.c_str());
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Service call failed.");
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EndCompetitionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
