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

// Callback function to read and save incoming orders
void ShipOrders::order_callback(const ariac_msgs::msg::Order::SharedPtr msg)
{
    // Push order to orders_ vector
    orders_.push_back(*msg);

    // Start the process_timer_ if we received the first order
    if (!first_order_received_) {
        first_order_received_ = true;

        process_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ShipOrders::process_orders, this));
        
        // process_timer_ = this->create_wall_timer(
        //     std::chrono::seconds(35),
        //     std::bind(&ShipOrders::lock_tray, this);
    }
}

// Sort and start shipping orders
void ShipOrders::process_orders()
{
    process_timer_->cancel();

    // Return if orders_ is empty
    if (orders_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Timer expired and no orders were received");
        return;
    }

    // Sort the orders
    std::sort(orders_.begin(), orders_.end(),
        [](const ariac_msgs::msg::Order &a, const ariac_msgs::msg::Order &b) {
            return a.priority > b.priority;
        });

    // Start processing the orders in the sorted vector
    current_order_index_ = 0;
    process_next_order();
}

// Locks the tray and moves the AVG for the orders
void ShipOrders::process_next_order()
{
    // Return once orders have been processed
    if (current_order_index_ >= orders_.size()) {
        RCLCPP_INFO(this->get_logger(), "Orders processed.");
        return;
    }

    const auto &order = orders_[current_order_index_];
    int agv_num = order.kitting_task.agv_number;       // agv id
    int destination = order.kitting_task.destination;  // order destination

    RCLCPP_INFO(this->get_logger(), "Shipping Order '%s' on AGV%d",
                order.id.c_str(), agv_num);

    // Lock the tray
    lock_tray(agv_num, destination);
}

void ShipOrders::lock_tray()
{

}
// Lock the tray for the current order in place
void ShipOrders::lock_tray(int agv_num, int destination)
{   
    process_timer_->cancel();

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
                current_order_index_++;
                this->process_next_order();
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
            
            // Add one to the order index and continue to the next order
            current_order_index_++;
            this->process_next_order();
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