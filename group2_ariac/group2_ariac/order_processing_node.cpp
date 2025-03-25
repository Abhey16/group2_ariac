#include "rclcpp/rclcpp.hpp"
#include "ariac_msgs/msg/order.hpp"
#include <queue>
#include <vector>
#include <mutex>

using std::placeholders::_1;

struct OrderComparator {
    bool operator()(const ariac_msgs::msg::Order &a, const ariac_msgs::msg::Order &b) const {
        return a.priority < b.priority;  
    }
};

class OrderManager : public rclcpp::Node {
public:
    OrderManager() : Node("order_manager") {
        order_sub_ = this->create_subscription<ariac_msgs::msg::Order>(
            "/ariac/orders", 10, std::bind(&OrderManager::order_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Order Manager Node Initialized");
    }

private:
    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_sub_;

    std::priority_queue<ariac_msgs::msg::Order,
                        std::vector<ariac_msgs::msg::Order>,
                        OrderComparator> order_queue_;

    std::mutex queue_mutex_;

    void order_callback(const ariac_msgs::msg::Order::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        RCLCPP_INFO(this->get_logger(), "Received Order: %s (Priority: %d)", msg->id.c_str(), msg->priority);

        order_queue_.push(*msg);

        RCLCPP_INFO(this->get_logger(), "Current Queue Size: %ld", order_queue_.size());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OrderManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
