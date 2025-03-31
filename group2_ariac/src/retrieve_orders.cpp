#include "retrieve_orders.hpp"

// Callback function to process incoming order messages
void RetrieveOrders::order_callback(const ariac_msgs::msg::Order::SharedPtr msg)
{

    // Get Order Attributes

    // Create a vector to store kitting parts
    std::vector<KittingPart> kitting_part;

    // Loop through all parts in the kitting task and create KittingPart objects
    for (const auto &part_msg : msg->kitting_task.parts)
    {
        kitting_part.push_back(KittingPart(Part(part_msg.part.color, part_msg.part.type),
                                           part_msg.quadrant));
    }

    // Create KittingTask object
    KittingTask kitting(msg->kitting_task.agv_number,
                        msg->kitting_task.tray_id,
                        msg->kitting_task.destination,
                        kitting_part);

    // Create AssemblyPart Object
    std::vector<AssemblyPart> assembly_part;
    for (const auto &part_msg : msg->assembly_task.parts)
    {
        assembly_part.push_back(AssemblyPart(Part(part_msg.part.color, part_msg.part.type),
                                             part_msg.assembled_pose,
                                             part_msg.install_direction));
    }

    // Create AssemblyTask object
    AssemblyTask assembly(msg->assembly_task.agv_numbers,
                          msg->assembly_task.station,
                          assembly_part);

    // Create CombinedPart object
    std::vector<AssemblyPart> combined_part;
    for (const auto &part_msg : msg->combined_task.parts)
    {
        combined_part.push_back(AssemblyPart(Part(part_msg.part.color, part_msg.part.type),
                                             part_msg.assembled_pose,
                                             part_msg.install_direction));
    }
    // Create CombinedTask object
    CombinedTask combined(msg->combined_task.station,
                          combined_part);

    // Create Order object
    Order order(msg->id,
                msg->type, msg->priority,
                kitting,
                assembly,
                combined);

    // Storing orders in a queue
    if (msg->priority == true)
    {
        priority_orders.push(order); // If it's a priority order, push it to the priority queue
    }
    else
    {
        normal_orders.push(order); // If it's a normal order, push it to the normal queue

    }

    // Display the order details
    RetrieveOrders::display_order(order);
}

void RetrieveOrders::display_order(const Order &order)
{
    // Display order on the terminal
    RCLCPP_INFO(this->get_logger(), "*****************************");
    RCLCPP_INFO(this->get_logger(), "- Kitting Task : %s", order.get_id().c_str());

    RCLCPP_INFO(this->get_logger(), "    - Priority : %d", order.get_priority());

    RCLCPP_INFO(this->get_logger(), "    - Tray : %d", order.get_kitting_task().get_tray_id());

    for (const auto &part : order.get_kitting_task().get_parts())
    {
        RCLCPP_INFO(this->get_logger(), "       - %s: %s %s",
                    part.get_quadrant().c_str(),
                    part.get_part().get_color().c_str(),
                    part.get_part().get_type().c_str());
    }

    RCLCPP_INFO(this->get_logger(), "    - AGV : %d", order.get_kitting_task().get_agv_number());

    RCLCPP_INFO(this->get_logger(), "*****************************");
}

// Main function to initialize ROS node and start spinning
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // Initialize ROS with command-line arguments

    // Create a RetrieveOrders node instance
    auto node = std::make_shared<RetrieveOrders>("retrieve_orders");

    // Start spinning the node to handle callbacks and events
    rclcpp::spin(node);
    
    // Shutdown ROS when done
    rclcpp::shutdown();

    return 0;
}
