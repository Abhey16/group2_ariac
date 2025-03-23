#include "rclcpp/rclcpp.hpp"
// #include <string>
// include order interface
# include "ariac_msgs/msg/order.hpp"

class RetrieveOrders : public rclcpp::Node
{   
    public:
        // Constructor
        RetrieveOrders(std::string node_name) : Node(node_name)
        {
            subscriber_ = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders",10,
                                                                            std::bind(&RetrieveOrders::order_callback, this,
                                                                            std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(),"subscriber created");
        }

        // subscriber callback
        void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);

    private:
        // subscriber_
        rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr subscriber_;

};