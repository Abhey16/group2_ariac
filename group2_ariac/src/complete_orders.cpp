#include "complete_orders.hpp"

    void CompleteOrders::order_callback(const ariac_msgs::msg::Order::SharedPtr msg) {
        std::string order_id = msg->id;
        int agv_num = msg->kitting_task.agv_number;

        agv_to_order_[agv_num] = order_id;

        RCLCPP_INFO(this->get_logger(),
                    "Received order '%s' assigned to AGV%d",
                    order_id.c_str(), agv_num);
    }

    void CompleteOrders::submit_order(const std::string &order_id) {
        auto client = this->create_client<ariac_msgs::srv::SubmitOrder>("/ariac/submit_order");

        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Service /ariac/submit_order not available");
            return;
        }

        auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
        request->order_id = order_id;

        auto future = client->async_send_request(request);

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(shared_from_this());
        executor.spin_until_future_complete(future);

        if (future.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Order '%s' submitted successfully!", order_id.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to submit order '%s'", order_id.c_str());
        }
    }

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CompleteOrders>());
    rclcpp::shutdown();
    return 0;
}
