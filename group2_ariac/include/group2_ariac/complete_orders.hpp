#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/agv_status.hpp>
#include <ariac_msgs/srv/submit_order.hpp>

#include <unordered_map>
#include <string>

class CompleteOrders : public rclcpp::Node
{
public:
    CompleteOrders() : Node("complete_orders")
    {
        order_sub_ = this->create_subscription<ariac_msgs::msg::Order>(
            "/ariac/orders", 10,
            std::bind(&CompleteOrders::order_callback, this, std::placeholders::_1));

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

        submit_client_ = this->create_client<ariac_msgs::srv::SubmitOrder>("/ariac/submit_order");

        RCLCPP_INFO(this->get_logger(), "Complete Orders Node Initialized");
    }

private:
    void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);
    void agv_status_callback(int agv_num, const ariac_msgs::msg::AGVStatus::SharedPtr msg);
    void submit_order(const std::string &order_id);

    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_sub_;
    std::unordered_map<int, rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr> agv_subs_;

    rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr submit_client_;
    std::unordered_map<int, ariac_msgs::msg::Order> orders_;
};