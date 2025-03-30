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
        // Subscribe to all AGV statuses
        for (int agv = 1; agv <= 4; ++agv) {
            std::string topic = "/ariac/agv" + std::to_string(agv) + "_status";

            agv_status_subs_[agv] = this->create_subscription<ariac_msgs::msg::AGVStatus>(
                topic, 10,
                [this, agv](const ariac_msgs::msg::AGVStatus::SharedPtr msg) {
                    agv_locations_[agv] = msg->location;

                    // If AGV reached warehouse and we know its order
                    if (agv_to_order_.count(agv) &&
                        !submitted_orders_.count(agv_to_order_[agv]) &&
                        msg->location == ariac_msgs::msg::AGVStatus::WAREHOUSE) {

                        std::string order_id = agv_to_order_[agv];
                        RCLCPP_INFO(this->get_logger(),
                                    "AGV%d reached warehouse. Submitting order '%s'...",
                                    agv, order_id.c_str());

                        submit_order(order_id);
                        submitted_orders_.insert(order_id);
                    }
                });
        }

        // Subscribe to /ariac/orders to track incoming orders and AGV mappings
        order_sub_ = this->create_subscription<ariac_msgs::msg::Order>(
            "/ariac/orders", 10,
            std::bind(&CompleteOrders::order_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Waiting for orders and AGV arrivals...");
    }

private:
    std::map<int, int> agv_locations_;
    std::map<int, std::string> agv_to_order_;
    std::set<std::string> submitted_orders_;   // submitted orders
    std::map<int, rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr> agv_status_subs_;
    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_sub_;

    void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);

    void submit_order(const std::string &order_id);
};