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
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
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
    ShipOrders() : Node("ship_orders"), ship_agv_(false), agv_number_(0)
    {
        // Subscriber for ship_agv
        ship_agv_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/group2_ariac/ship_agv", 10,
            std::bind(&ShipOrders::ship_agv_cb, this, std::placeholders::_1));

        // Subscriber for current_agv
        current_agv_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/group2_ariac/current_agv", 10,
            std::bind(&ShipOrders::current_agv_cb, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ShipOrders::ship_orders_timer, this));

        RCLCPP_INFO(this->get_logger(), "Ship Orders Node Initialized");
    }

private:
    /**
     * @brief Callback for group2_ariac/ship_agv subscribtion
     */
    void ship_agv_cb(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Callback for group2_ariac/current_agv subscribtion
     */
    void current_agv_cb(const std_msgs::msg::Int8::SharedPtr msg);    

    /**
     * @brief Method used to ship the orders
     */
    void ship_orders_timer();

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
     * @brief Subscription to group2_ariac/ship_agv
     */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ship_agv_sub_;

    /**
     * @brief Subscription to group2_ariac/current_agv
     */
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr current_agv_sub_;

    /**
     * @brief 
     */
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief Clients to start lock services
     */
    std::unordered_map<int, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> tray_clients_;

    /**
     * @brief Clients to start move services
     */
    std::unordered_map<int, rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr> move_clients_;

    /**
     * @brief Flag to start moving agv
     */
    bool ship_agv_;
    
    /**
     * @brief agv id
     */
    int agv_number_;

}; // class ShipOrders