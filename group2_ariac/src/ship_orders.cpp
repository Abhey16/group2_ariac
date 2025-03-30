#include "ship_orders.hpp"

 bool ShipOrders::lock_agv_tray(int agv_num) {
        std::string service_name = "/ariac/agv" + std::to_string(agv_num) + "_lock_tray";
        auto client = this->create_client<std_srvs::srv::Trigger>(service_name);

        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Service %s not available", service_name.c_str());
            return false;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future)
            == rclcpp::FutureReturnCode::SUCCESS && future.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Locked tray on AGV%d", agv_num);
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to lock tray on AGV%d", agv_num);
            return false;
        }
    }

    bool ShipOrders::move_agv(int agv_num, int destination) {
        std::string service_name = "/ariac/move_agv" + std::to_string(agv_num);
        auto client = this->create_client<ariac_msgs::srv::MoveAGV>(service_name);

        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Service %s not available", service_name.c_str());
            return false;
        }
        auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
        request->location = destination;

        auto future = client->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

        int timeout = 22;
        auto start_time = std::chrono::steady_clock::now();

        while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(timeout)) {
            if (agv_locations_[agv_num] == destination) {
                RCLCPP_INFO(this->get_logger(), "Moved AGV%d to destination %d", agv_num, destination);
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        RCLCPP_WARN(this->get_logger(), "Timeout: AGV%d did not reach destination %d", agv_num, destination);
        return false;
    }

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShipOrders>());
    rclcpp::shutdown();
    return 0;
}
