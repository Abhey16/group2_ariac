#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/srv/move_agv.hpp>

#include <vector>
#include <unordered_map>
#include <memory>

class ShipOrders : public rclcpp::Node
{
public:
    ShipOrders() : Node("ship_orders"), current_order_index_(0), first_order_received_(false)
    {
        order_sub_ = this->create_subscription<ariac_msgs::msg::Order>(
            "/ariac/orders", 10,
            std::bind(&ShipOrders::order_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Ship Orders Node Initialized");
    }

private:
    void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);
    void process_orders();
    void process_next_order();
    void lock_tray(int agv_num, int destination);
    void move_agv(int agv_num, int destination);

    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_sub_;
    rclcpp::TimerBase::SharedPtr process_timer_;

    std::unordered_map<int, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> tray_clients_;
    std::unordered_map<int, rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr> move_clients_;

    std::vector<ariac_msgs::msg::Order> orders_;
    size_t current_order_index_;
    bool first_order_received_;
    
}; // class ShipOrders