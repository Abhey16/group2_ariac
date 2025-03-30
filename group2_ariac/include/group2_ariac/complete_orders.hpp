#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ariac_msgs/msg/agv_status.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/srv/submit_order.hpp>

#include <map>
#include <set>
#include <string>

class CompleteOrders : public rclcpp::Node {
public:
    CompleteOrders() : Node("complete_orders_node") {

        order_sub_ = this->create_subscription<ariac_msgs::msg::Order>(
            "/ariac/orders", 10,
            std::bind(&CompleteOrders::order_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Complete Orders Node Initialized");
    }

private:
    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_sub_;
    std::vector<ariac_msgs::msg::Order> orders_;    

    void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);

    void submit_order(const std::string &order_id);
};