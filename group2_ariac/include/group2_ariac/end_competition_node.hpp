#pragma once

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

    void state_callback(const ariac_msgs::msg::CompetitionState::SharedPtr msg);
};