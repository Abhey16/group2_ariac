/**
 * @file ship_orders.cpp
 * @author Rey Roque-Perez (reyroque@umd.edu)
 * @brief Implementation of the Ship Orders class
 * @version 0.1
 * @date 2025-03-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "ship_orders.hpp"

void ShipOrders::ship_agv_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
    ship_agv_ = msg->data;
}

void ShipOrders::current_agv_cb(const std_msgs::msg::Int8::SharedPtr msg)
{
    agv_number_ = msg->data;
}

// Ship orders
void ShipOrders::ship_orders_timer()
{
    if (ship_agv_ && agv_number_ != 0)
    {    
        int agv_num = agv_number_;       // agv id
        int destination = 3;  // warehouse

        RCLCPP_INFO(this->get_logger(), "Moving AGV%d to warehouse",
                    agv_num);
        
        // Lock the tray
        lock_tray(agv_num, destination);
        ship_agv_ = false;
        agv_number_ = 0;
    }
}

// Lock the tray for the current order in place
void ShipOrders::lock_tray(int agv_num, int destination)
{   
    // Check if service exists and creates one if necessary
    if (tray_clients_.find(agv_num) == tray_clients_.end()) {
        std::string service_name = "/ariac/agv" + std::to_string(agv_num) + "_lock_tray";
        tray_clients_[agv_num] = this->create_client<std_srvs::srv::Trigger>(service_name);
    }
    auto tray_client = tray_clients_[agv_num];

    // Return if we don't get a response after some time
    if (!tray_client->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(), "Service /ariac/agv%d_lock_tray timed out", agv_num);
        return;
    }

    // Sending request to lock the tray
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    tray_client->async_send_request(request,
        [this, agv_num, destination](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            if (future.get()->success) // Succesfully locked the tray
            {
                RCLCPP_INFO(this->get_logger(), "Locked tray on AGV%d", agv_num);
                // Add a short delay to ensure plugin finishes locking
                rclcpp::sleep_for(std::chrono::milliseconds(500));
                this->move_agv(agv_num, destination);
            } else // Failed to lock tray 
            {
                RCLCPP_WARN(this->get_logger(), "Failed to lock tray on AGV%d", agv_num);
            }
        });
}

// Move the AGV to the order destination
void ShipOrders::move_agv(int agv_num, int destination)
{
    // Check if service exists and creates one if necessary
    if (move_clients_.find(agv_num) == move_clients_.end()) {
        std::string service_name = "/ariac/move_agv" + std::to_string(agv_num);
        move_clients_[agv_num] = this->create_client<ariac_msgs::srv::MoveAGV>(service_name);
    }
    auto client = move_clients_[agv_num];

    // Return if we don't get a response after some time
    if (!client->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(), "Service /ariac/move_agv%d timed out", agv_num);
        return;
    }

    // Sending request to move the AGV
    auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
    request->location = destination;
    client->async_send_request(request,
        [this, agv_num, destination](rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedFuture future) {
            if (future.get()->success) // Moved succesfully
            {
                RCLCPP_INFO(this->get_logger(), "Moved AGV%d to destination %d", agv_num, destination);
            } else  // Move failed 
            {
                RCLCPP_WARN(this->get_logger(), "Move AGV%d to destination failed %d", agv_num, destination);
            }
        });
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ShipOrders>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}