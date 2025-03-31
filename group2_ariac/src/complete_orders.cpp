/**
 * @file ship_orders.cpp
 * @author Rey Roque-Perez (reyroque@umd.edu)
 * @brief Implementation of the Complete Orders class
 * @version 0.1
 * @date 2025-03-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "complete_orders.hpp"

// Callback function to read and save incoming orders
void CompleteOrders::order_callback(const ariac_msgs::msg::Order::SharedPtr msg)
{
    orders_[msg->kitting_task.agv_number] = *msg;
}

// Callback function to get agv status updates
void CompleteOrders::agv_status_callback(int agv_num, const ariac_msgs::msg::AGVStatus::SharedPtr msg)
{
    // Submit the order and remove from list if it has reached the warehouse
    if (msg->location == 3 && orders_.count(agv_num)) {
        submit_order(orders_[agv_num].id);
        orders_.erase(agv_num);  // remove order from the list
    }
}

// Method used to submit the order
void CompleteOrders::submit_order(const std::string &order_id)
{
    // Return if we don't get a response after some time
    if (!submit_client_->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(), "Submit order service timed out");
        return;
    }

    // Sending request to submit the order
    auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
    request->order_id = order_id;
    submit_client_->async_send_request(request,
        [this, order_id](rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedFuture future) {
            if (future.get()->success) // Order submitted succesfully 
            {
                RCLCPP_INFO(this->get_logger(), "Submitted order: %s", order_id.c_str());
                if (orders_.empty()) {
                    std_msgs::msg::Bool completed_msg;
                    completed_msg.data = true;
                    orders_completed_pub_->publish(completed_msg);
                }
            } else  // Order failed to submit 
            {
                RCLCPP_WARN(this->get_logger(), "Failed to submit order: %s", order_id.c_str());
            }
        });
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CompleteOrders>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}