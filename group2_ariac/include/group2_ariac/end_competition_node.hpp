/**
 * @file end_competition_node.hpp
 * @author Wei-Li, Chen (wc2023@umd.com)
 * @brief Class definition for the EndCompetitionNode class
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
#include <std_msgs/msg/bool.hpp>

using std::placeholders::_1;

/**
 * @class EndCompetitionNode
 * @brief ends the competition once all orders are completed
 */
class EndCompetitionNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a new End Competition Node object.
     */
    EndCompetitionNode() : Node("end_competition_node") {
        
        // end competition client
        end_competition_client_ = this->create_client<std_srvs::srv::Trigger>("/ariac/end_competition");

        // subsribe competition state
        state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>(
            "/ariac/competition_state", 10,
            std::bind(&EndCompetitionNode::state_callback, this, _1));
        
        // subscribe orders completed flag
        orders_completed_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/ariac/orders_completed", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                orders_completed_ = msg->data;
            });
        
        // log message
        RCLCPP_INFO(this->get_logger(), "End Competition Node Initialized");
    }

private:
    /**
     * @brief Callback function to read incoming competition state
     */
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr state_sub_;
    
    /**
     * @brief Callback function to read incoming orders completed flag
     */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr orders_completed_sub_;    
    
    /**
     * @brief Client for calling /ariac/end_competition
     */
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr end_competition_client_;

    /**
     * @brief Flag to indicate if all orders have been completed
     */
    bool orders_completed_ = false;
    
    /**
     * @brief Flag to ensure competition is only ended once
     */
    bool ended_ = false;
    
    /**
     * @brief  Callback function to read incoming competition state
     * 
     * @param msg  [const ariac_msgs::msg::CompetitionState::SharedPtr] Competition state message
     */
    void state_callback(const ariac_msgs::msg::CompetitionState::SharedPtr msg);
};