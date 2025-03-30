#include "complete_orders.hpp"

void CompleteOrders::order_callback(const ariac_msgs::msg::Order::SharedPtr msg)
{
    orders_[msg->kitting_task.agv_number] = *msg;
}

void CompleteOrders::agv_status_callback(int agv_num, const ariac_msgs::msg::AGVStatus::SharedPtr msg)
{
    if (msg->location == 3 && orders_.count(agv_num)) {
        submit_order(orders_[agv_num].id);
        orders_.erase(agv_num);  // remove order from list
    }
}

void CompleteOrders::submit_order(const std::string &order_id)
{
    if (!submit_client_->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(), "Submit order service timed out");
        return;
    }

    auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
    request->order_id = order_id;

    submit_client_->async_send_request(request,
        [this, order_id](rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedFuture future) {
            if (future.get()->success) {
                RCLCPP_INFO(this->get_logger(), "Submitted order: %s", order_id.c_str());
            } else {
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