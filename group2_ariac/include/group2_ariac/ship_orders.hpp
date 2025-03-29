#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/agv_status.hpp>
#include <ariac_msgs/srv/move_agv.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <map>
#include <algorithm>
#include <chrono>
#include <thread>

class ShipOrders : public rclcpp::Node {
public:
    // Constructor
    ShipOrders() : Node("ship_orders") {
        // Initializing AGV status subscribers
        for (int agv = 1; agv <= 4; ++agv) {
            std::string topic = "/ariac/agv" + std::to_string(agv) + "_status";

            agv_status_subs_[agv] = this->create_subscription<ariac_msgs::msg::AGVStatus>(
                topic, 10,
                [this, agv](const ariac_msgs::msg::AGVStatus::SharedPtr msg) {
                    agv_locations_[agv] = msg->location;
                });
        }

        // Sort orders by priority (HIGH first)
        std::sort(orders_.begin(), orders_.end(),
        [](const ariac_msgs::msg::Order &a, const ariac_msgs::msg::Order &b) {
            return a.priority > b.priority;  // true > false → High priority first
        });

        for (const auto &order : orders_) {
            int agv_num = order.kitting_task.agv_number;

            if (lock_agv_tray(agv_num)) {
                move_agv(agv_num, ariac_msgs::msg::KittingTask::WAREHOUSE);   // WAREHOUSE == 3
            }
        }
    }

private:
    // fields
    std::map<int, int> agv_locations_;
    std::map<int, rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr> agv_status_subs_;

    // Orders for testing
    std::vector<ariac_msgs::msg::Order> orders_ = {
        [] {
            ariac_msgs::msg::Order o;
            o.id = "order_1";
            o.priority = false;
            o.kitting_task.agv_number = 1;
            return o;
        }(),
        [] {
            ariac_msgs::msg::Order o;
            o.id = "order_2";
            o.priority = true;
            o.kitting_task.agv_number = 2;
            return o;
        }()
    };

    // Locks trays and parts on AGV
    bool lock_agv_tray(int agv_num);

    // Moves AGV n to a station
    bool move_agv(int agv_num, int destination);
    
}; // class ShipOrders