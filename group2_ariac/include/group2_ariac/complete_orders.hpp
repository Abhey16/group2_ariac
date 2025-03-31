/**
 * @file ship_orders.hpp
 * @author Rey Roque-Perez (reyroque@umd.com)
 * @brief Class definition for the Complete Orders class
 * @version 0.1
 * @date 2025-03-30
 *
 * @copyright Copyright (c) 2025
 *
 */

 #pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/agv_status.hpp>
#include <ariac_msgs/srv/submit_order.hpp>

#include <unordered_map>
#include <string>

/**
 * @brief Class that sumbits orders once they reach the warehouse
 */
class CompleteOrders : public rclcpp::Node
{
public:
    CompleteOrders() : Node("complete_orders")
    {
        // Subscriber for published orders
        order_sub_ = this->create_subscription<ariac_msgs::msg::Order>(
            "/ariac/orders", 10,
            std::bind(&CompleteOrders::order_callback, this, std::placeholders::_1));

        // Subscribers for AGV status
        for (int i = 1; i <= 4; ++i)
        {
            std::string topic = "/ariac/agv" + std::to_string(i) + "_status";
            agv_subs_[i] = this->create_subscription<ariac_msgs::msg::AGVStatus>(
                topic, 10,
                [this, i](const ariac_msgs::msg::AGVStatus::SharedPtr msg)
                {
                    this->agv_status_callback(i, msg);
                });
        }

        // Client for submitting orders
        submit_client_ = this->create_client<ariac_msgs::srv::SubmitOrder>("/ariac/submit_order");

        RCLCPP_INFO(this->get_logger(), "Complete Orders Node Initialized");
    }

private:
    /**
     * @brief Callback function to read incoming orders
     * 
     * @param msg [const ariac_msgs::msg::Order::SharedPtr] Order message
     */
    void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);

    /**
     * @brief Callback function to read AGV status
     * 
     * @param agv_num [int] AGV Id number
     * @param msg [const ariac_msgs::msg::AGVStatus::SharedPtr] AGV status message
     */
    void agv_status_callback(int agv_num, const ariac_msgs::msg::AGVStatus::SharedPtr msg);

    /**
     * @brief Method used to submit the completed orders
     * 
     * @param order_id [const std::string &] Order ID
     */
    void submit_order(const std::string &order_id);

    /**
     * @brief Subscription to ariac/orders
     */    
    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_sub_;
    /**
     * @brief Subscription to ariac/AGVStatus
     */    
    std::unordered_map<int, rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr> agv_subs_;

    /**
     * @brief Client used to sumbit the completed orders
     */
    rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr submit_client_;

    /**
     * @brief Map to store orders and their corresponding AGV
     */
    std::unordered_map<int, ariac_msgs::msg::Order> orders_;
};