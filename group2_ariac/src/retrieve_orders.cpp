#include "group2_ariac/retrieve_orders.hpp"

void RetrieveOrders::order_callback(const ariac_msgs::msg::Order::SharedPtr msg)
{

    // Get Order Attributes

    // Create parts object
    std::vector<KittingPart> kitting_part;
    for(const auto& part_msg : msg->kitting_task.parts)
    {
        kitting_part.push_back(KittingPart(Part(part_msg.part.color,part_msg.part.type),
                                    part_msg.quadrant));
    }

    // Create KittingTask object
    KittingTask kitting(msg->kitting_task.agv_number,
                        msg->kitting_task.tray_id,
                        msg->kitting_task.destination,
                        kitting_part);

    // Create Order object
    Order order(msg->id, 
                msg->type,msg->priority,
                kitting);

    // Storing orders in a queue
    orders.push(order);
    
    // Display order on the terminal
    RCLCPP_INFO(this->get_logger(),"*****************************");

    RCLCPP_INFO(this->get_logger(),"- Kitting Task : %s",order.get_id().c_str());

    RCLCPP_INFO(this->get_logger(),"    - Priority : %d", order.get_priority());

    RCLCPP_INFO(this->get_logger(),"    - Tray : %d",order.get_kitting_task().get_tray_id());
    
    for (const auto& part : order.get_kitting_task().get_parts()) {
        RCLCPP_INFO(this->get_logger(), "       - Q%d: %d %d",
            part.get_quadrant(),
            part.get_part().get_color(), 
            part.get_part().get_type()
        );
    }

    RCLCPP_INFO(this->get_logger(),"    - AGV : %d", order.get_kitting_task().get_agv_number());

    RCLCPP_INFO(this->get_logger(),"*****************************");
    
}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<RetrieveOrders>("retrieve_orders");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
