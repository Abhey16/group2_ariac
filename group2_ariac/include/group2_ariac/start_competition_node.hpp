/**
 * @file ship_orders.hpp
 * @author Wei-Li, Chen (wc2023@umd.com)
 * @brief Class definition for the Ship Orders class
 * @version 0.1
 * @date 2025-03-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ariac_msgs/msg/competition_state.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
/**
 * @class CompetitionManager
 * @brief Class that manages the competition state
 * 
 */
class CompetitionManager : public rclcpp::Node {
public:
    CompetitionManager() : Node("competition_manager") 
    {
        // Subscriber for competition state
        competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>(
            "/ariac/competition_state", 10, std::bind(&CompetitionManager::competition_state_callback, this, _1));

        start_competition_client_ = this->create_client<std_srvs::srv::Trigger>("/ariac/start_competition");

        RCLCPP_INFO(this->get_logger(), "Competition Manager Node Initialized");
    }

private:
    /**
     * @brief  Callback function to read incoming competition state
     * 
     */
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_sub_;
    
    /**
     * @brief  Client for starting the competition
     * 
     */
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_competition_client_;

    /**
     * @brief Callback function to read incoming competition state
     * 
     * @param msg Competition state message
     */
    void competition_state_callback(const ariac_msgs::msg::CompetitionState::SharedPtr msg);
};