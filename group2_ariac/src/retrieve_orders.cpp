#include "group2_ariac/retrieve_orders.hpp"

void RetrieveOrders::order_callback(const ariac_msgs::msg::Order::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(),"callback");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<RetrieveOrders>("retrieve_orders");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
