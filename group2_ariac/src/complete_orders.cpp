#include "complete_orders.hpp"

void CompleteOrders::order_callback(const ariac_msgs::msg::Order::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received Order ID: %s | Priority: %s | AGV: %d",
                msg->id.c_str(),
                msg->priority ? "HIGH" : "NORMAL",
                msg->kitting_task.agv_number);

    orders_.push_back(*msg);
}

    void CompleteOrders::submit_order(const std::string &order_id) {
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
