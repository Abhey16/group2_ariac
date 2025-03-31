/**
 * @file ship_orders.hpp
 * @author Rey Roque-Perez (reyroque@umd.com)
 * @brief Class definition for the Ship Orders class
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
#include <ariac_msgs/srv/move_agv.hpp>

#include <vector>
#include <unordered_map>
#include <memory>

/**
 * @brief Class that ships orders by locking and moving the required AGV
 */
class ShipOrders : public rclcpp::Node
{
public:
    ShipOrders() : Node("ship_orders"), current_order_index_(0), first_order_received_(false)
    {
        // Subscriber for published orders
        order_sub_ = this->create_subscription<ariac_msgs::msg::Order>(
            "/ariac/orders", 10,
            std::bind(&ShipOrders::order_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Ship Orders Node Initialized");
    }

private:
    /**
     * @brief Callback function to read incoming orders
     * 
     * @param msg [const ariac_msgs::msg::Order::SharedPtr] Order message
     */
    void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);

    /**
     * @brief Method used to process the orders vector
     */
    void process_orders();

    /**
     * @brief Method to process an individual order
     */
    void process_next_order();

    /**
     * @brief Method to lock an AGV's tray
     * 
     * @param agv_num [int] AGV Id number
     * @param destination [int] move destination
     */
    void lock_tray(int agv_num, int destination);

    /**
     * @brief Method to move an AGV
     * 
     * @param agv_num [int] AGV Id number
     * @param destination [int] move destination
     */    
    void move_agv(int agv_num, int destination);

    /**
     * @brief Subscription to ariac/orders
     */
    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_sub_;

    /**
     * @brief Timer used to start processing the orders
     */
    rclcpp::TimerBase::SharedPtr process_timer_;

    /**
     * @brief Clients to start lock services
     */
    std::unordered_map<int, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> tray_clients_;

    /**
     * @brief Clients to start move services
     */
    std::unordered_map<int, rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr> move_clients_;

    /**
     * @brief Vector to store order messages
     */
    std::vector<ariac_msgs::msg::Order> orders_;

    /**
     * @brief Stores the index of the current order being processed
     */
    size_t current_order_index_;

    /**
     * @brief Flag to start processing orders
     */
    bool first_order_received_;
    
}; // class ShipOrders