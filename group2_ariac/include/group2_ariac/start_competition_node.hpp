#pragma once

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

    void competition_state_callback(const ariac_msgs::msg::CompetitionState::SharedPtr msg);
};