#include "end_competition_node.hpp"
void EndCompetitionNode::state_callback(const ariac_msgs::msg::CompetitionState::SharedPtr msg)
{
    if (ended_)
        return;

    if (msg->competition_state == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE)
    {
        RCLCPP_INFO(this->get_logger(), "Competition state is ORDER_ANNOUNCEMENTS_DONE. Ending competition...");

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        while (!end_competition_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for /ariac/end_competition service...");
        }

        auto future = end_competition_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            if (future.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "Competition ended successfully.");
                ended_ = true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to end competition: %s", future.get()->message.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EndCompetitionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
